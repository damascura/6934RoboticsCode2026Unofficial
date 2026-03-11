package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.Constants.QuickTuning;
import frc.robot.Constants.Vision;
import frc.robot.commands.Load;
import frc.robot.commands.RunIntakeRollers;
import frc.robot.commands.Shoot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ToggleIntakePivot;
import frc.robot.commands.VisionAutoAlign;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.ShooterSubsys;
import frc.robot.subsystems.Swerve;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private int activePathfindingCommandCount = 0;

    /* Controllers */
    private final Joystick driver = new Joystick(QuickTuning.driveControllerID);
    private final Joystick weapons = new Joystick(QuickTuning.weaponControllerID);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton toggleSlowMode = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driveToTagPrimary = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton driveToTagSecondary = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton driveToTagTertiary = new JoystickButton(driver, XboxController.Button.kBack.value);

    private final JoystickButton autoAlignCenter = new JoystickButton(weapons, XboxController.Button.kA.value);
    private final JoystickButton autoAlignLeft = new JoystickButton(weapons, XboxController.Button.kX.value);
    private final JoystickButton autoAlignRight = new JoystickButton(weapons, XboxController.Button.kY.value);
    private final JoystickButton intakePivotToggle = new JoystickButton(weapons, XboxController.Button.kLeftBumper.value);
    private final JoystickButton intakeRollers = new JoystickButton(weapons, XboxController.Button.kRightBumper.value);
    private final JoystickButton load = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton shoot = new JoystickButton(driver, XboxController.Button.kA.value);
    private final NetworkButton elasticAutoAlignCenter;
    private final NetworkButton elasticAutoAlignLeft;
    private final NetworkButton elasticAutoAlignRight;

    /* Elastic Topics */
    private final StringEntry elasticSelectedAutoEntry;
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ShooterSubsys s_Shooter = new ShooterSubsys();
    private final Loader s_Loader = new Loader();
    private final Intake s_Intake = new Intake();
    private final Command driveToTagPrimaryCommand = createDriveToTagPathCommand(Vision.driveToTagPrimaryTagId, "Primary");
    private final Command driveToTagSecondaryCommand = createDriveToTagPathCommand(Vision.driveToTagSecondaryTagId, "Secondary");
    private final Command driveToTagTertiaryCommand = createDriveToTagPathCommand(Vision.driveToTagTertiaryTagId, "Tertiary");

    public RobotContainer() {
        NetworkTable elasticTable = NetworkTableInstance.getDefault().getTable("Elastic");
        elasticSelectedAutoEntry = elasticTable.getStringTopic("Auto/Selected").getEntry("");

        List<String> autoOptions = loadElasticAutoOptions();
        String defaultAuto = autoOptions.contains("RHubShoot") ? "RHubShoot" : "DoNothing";
        autoChooser.setDefaultOption(defaultAuto, defaultAuto);
        for (String autoName : autoOptions) {
            if (!defaultAuto.equals(autoName)) {
                autoChooser.addOption(autoName, autoName);
            }
        }
        SmartDashboard.putData("Auto Chooser", autoChooser);
        elasticSelectedAutoEntry.set(defaultAuto);

        elasticAutoAlignCenter = new NetworkButton(elasticTable, "Commands/AutoAlignCenter");
        elasticAutoAlignLeft = new NetworkButton(elasticTable, "Commands/AutoAlignLeft");
        elasticAutoAlignRight = new NetworkButton(elasticTable, "Commands/AutoAlignRight");

        registerNamedCommands();

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> robotCentric.getAsBoolean()
            )
        );

        registerStatusSendables();
        configureButtonBindings();
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("Load", new Load(s_Loader));
        NamedCommands.registerCommand("Shoot", new Shoot(s_Shooter));
        NamedCommands.registerCommand("Run Intake Rollers", new RunIntakeRollers(s_Intake));
        NamedCommands.registerCommand("Toggle Intake Pivot", new ToggleIntakePivot(s_Intake));
        NamedCommands.registerCommand("Auto Align Center", new VisionAutoAlign(s_Swerve, Vision.autoAlignCenterOffsetX));
        NamedCommands.registerCommand("Auto Align Left", new VisionAutoAlign(s_Swerve, Vision.autoAlignLeftOffsetX));
        NamedCommands.registerCommand("Auto Align Right", new VisionAutoAlign(s_Swerve, Vision.autoAlignRightOffsetX));
    }

    private void registerStatusSendables() {
        SmartDashboard.putBoolean("Pathfinding Active", false);

        SmartDashboard.putData("Vision Status", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("RobotPreferences");
                builder.addBooleanProperty("Valid AprilTag Detected", VisionInfo::hasValidTargets, null);
            }
        });

        SmartDashboard.putData("Robot Status", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("RobotPreferences");
                builder.addBooleanProperty("Autonomous Enabled", DriverStation::isAutonomousEnabled, null);
                builder.addBooleanProperty("AutoAlign Active", VisionAutoAlign::isAnyAutoAlignActive, null);
                builder.addBooleanProperty("Pathfinding Active", this::isPathfindingActive, null);
                builder.addBooleanProperty(
                    "Autonomous Or AutoAlign",
                    () -> DriverStation.isAutonomousEnabled() || VisionAutoAlign.isAnyAutoAlignActive() || isPathfindingActive(),
                    null
                );
            }

            private boolean isPathfindingActive() {
                return activePathfindingCommandCount > 0;
            }
        });
    }

    private List<String> loadElasticAutoOptions() {
        List<String> autos = new ArrayList<>();
        autos.add("DoNothing");

        Path autosDir = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("autos");
        try {
            List<String> discovered = Files.list(autosDir)
                .filter(path -> path.toString().endsWith(".auto"))
                .map(path -> path.getFileName().toString().replace(".auto", ""))
                .sorted()
                .collect(Collectors.toList());
            autos.addAll(discovered);
        } catch (Exception e) {
            Logger.recordOutput("Elastic/Auto/OptionsLoadError", e.getMessage());
        }

        return autos;
    }

    private void configureButtonBindings() {
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        toggleSlowMode.onTrue(Commands.runOnce(() -> {
            boolean isSlow = s_Swerve.getSpeedMultiplier() == QuickTuning.driveSlowModeMultiplier;
            if (isSlow) {
                s_Swerve.setSpeedMultiplier(1);
            } else {
                s_Swerve.setSpeedMultiplier(QuickTuning.driveSlowModeMultiplier);
            }
        }));

        autoAlignCenter.whileTrue(new VisionAutoAlign(s_Swerve, Vision.autoAlignCenterOffsetX));
        autoAlignLeft.whileTrue(new VisionAutoAlign(s_Swerve, Vision.autoAlignLeftOffsetX));
        autoAlignRight.whileTrue(new VisionAutoAlign(s_Swerve, Vision.autoAlignRightOffsetX));
        intakePivotToggle.onTrue(new ToggleIntakePivot(s_Intake));
        intakeRollers.whileTrue(new RunIntakeRollers(s_Intake));
        driveToTagPrimary.whileTrue(driveToTagPrimaryCommand);
        driveToTagSecondary.whileTrue(driveToTagSecondaryCommand);
        driveToTagTertiary.whileTrue(driveToTagTertiaryCommand);
        shoot.whileTrue(new Shoot(s_Shooter));
        load.whileTrue(new Load(s_Loader));
        load.onFalse(Commands.startEnd(s_Loader::runReverse, s_Loader::stop, s_Loader)
            .withTimeout(Constants.Loader.reverseSeconds));

        elasticAutoAlignCenter.whileTrue(new VisionAutoAlign(s_Swerve, Vision.autoAlignCenterOffsetX));
        elasticAutoAlignLeft.whileTrue(new VisionAutoAlign(s_Swerve, Vision.autoAlignLeftOffsetX));
        elasticAutoAlignRight.whileTrue(new VisionAutoAlign(s_Swerve, Vision.autoAlignRightOffsetX));
    }

    public Command getAutonomousCommand() {
        s_Swerve.setSpeedMultiplier(1);
        String selectedAuto = autoChooser.getSelected();
        if (selectedAuto == null || selectedAuto.isEmpty()) {
            selectedAuto = "DoNothing";
        }
        elasticSelectedAutoEntry.set(selectedAuto);
        Logger.recordOutput("Elastic/Auto/Selected", selectedAuto);

        if ("DoNothing".equals(selectedAuto)) {
            return null;
        }

        try {
            return new PathPlannerAuto(selectedAuto);
        } catch (Exception e) {
            Logger.recordOutput("Elastic/Auto/LoadError", "Failed to load auto: " + selectedAuto);
            return null;
        }
    }

    private Command createDriveToTagPathCommand(int tagId, String slotName) {
        return Commands.defer(() -> {
            Optional<edu.wpi.first.math.geometry.Pose3d> maybeTagPose =
                Vision.fieldLayout.getTagPose(tagId);

            if (maybeTagPose.isEmpty()) {
                Logger.recordOutput("Vision/DriveToTagPath/" + slotName + "/HasTagPose", false);
                Logger.recordOutput("Vision/DriveToTagPath/" + slotName + "/TagId", tagId);
                return Commands.none();
            }

            Pose2d tagPose2d = maybeTagPose.get().toPose2d();
            Pose2d targetPose = tagPose2d.transformBy(new Transform2d(
                Vision.driveToTagFrontOffsetMeters,
                0.0,
                new Rotation2d()
            ));
            Rotation2d targetHeading = tagPose2d.getRotation().plus(Rotation2d.fromDegrees(180));
            targetPose = new Pose2d(targetPose.getTranslation(), targetHeading);

            PathConstraints constraints = new PathConstraints(
                Vision.driveToTagPathMaxVelocityMPS,
                Vision.driveToTagPathMaxAccelerationMPSSq,
                Vision.driveToTagPathMaxAngularVelocityRadPerSec,
                Vision.driveToTagPathMaxAngularAccelerationRadPerSecSq
            );

            Logger.recordOutput("Vision/DriveToTagPath/" + slotName + "/HasTagPose", true);
            Logger.recordOutput("Vision/DriveToTagPath/" + slotName + "/TagId", tagId);
            Logger.recordOutput("Vision/DriveToTagPath/" + slotName + "/TargetPose", targetPose);
            return AutoBuilder.pathfindToPose(targetPose, constraints, Vision.driveToTagPathGoalEndVelocityMPS);
        }, Set.<Subsystem>of(s_Swerve))
            .beforeStarting(() -> setPathfindingActive(true))
            .finallyDo(_interrupted -> setPathfindingActive(false));
    }

    private void setPathfindingActive(boolean active) {
        if (active) {
            activePathfindingCommandCount++;
        } else {
            activePathfindingCommandCount = Math.max(0, activePathfindingCommandCount - 1);
        }

        boolean isActive = activePathfindingCommandCount > 0;
        SmartDashboard.putBoolean("Pathfinding Active", isActive);
        Logger.recordOutput("Pathfinding Active", isActive);
    }
}
