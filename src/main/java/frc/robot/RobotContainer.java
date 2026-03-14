package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.QuickTuning;
import frc.robot.commands.Load;
import frc.robot.commands.ManualIntakePivot;
import frc.robot.commands.MoveIntakePivotTo;
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
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton toggleSlowMode = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    private final JoystickButton hubAutoAlign = new JoystickButton(weapons, XboxController.Button.kX.value);
    private final POVButton intakePivotUp = new POVButton(weapons, 0);
    private final POVButton intakePivotDown = new POVButton(weapons, 180);
    private final POVButton intakePivotMid = new POVButton(weapons, 270);
    private final JoystickButton intakeRollers = new JoystickButton(weapons, XboxController.Button.kRightBumper.value);
    private final JoystickButton load = new JoystickButton(weapons, XboxController.Button.kB.value);
    private final JoystickButton shoot = new JoystickButton(weapons, XboxController.Button.kLeftBumper.value);
    private final NetworkButton elasticHubAutoAlign;

    /* Elastic Topics */
    private final StringEntry elasticSelectedAutoEntry;
    private final StringArrayPublisher elasticAutoOptionsPublisher;
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ShooterSubsys s_Shooter = new ShooterSubsys();
    private final Loader s_Loader = new Loader();
    private final Intake s_Intake = new Intake();

    public RobotContainer() {
        NetworkTable elasticTable = NetworkTableInstance.getDefault().getTable("Elastic");
        elasticSelectedAutoEntry = elasticTable.getStringTopic("Auto/Selected").getEntry("");
        elasticAutoOptionsPublisher = elasticTable.getStringArrayTopic("Auto/Options").publish();

        List<String> autoOptions = loadElasticAutoOptions();
        String defaultAuto = autoOptions.contains("RHubShoot") ? "RHubShoot" : "BNzbInBShBC1BNzbInBShBC2BNzbInC3";
        autoChooser.setDefaultOption(defaultAuto, defaultAuto);
        for (String autoName : autoOptions) {
            if (!defaultAuto.equals(autoName)) {
                autoChooser.addOption(autoName, autoName);
            }
        }
        SmartDashboard.putData("Auto Chooser", autoChooser);
        elasticAutoOptionsPublisher.set(autoOptions.toArray(new String[0]));
        if (elasticSelectedAutoEntry.get().isEmpty()) {
            elasticSelectedAutoEntry.set(defaultAuto);
        }

        elasticHubAutoAlign = new NetworkButton(elasticTable, "Commands/HubAutoAlign");

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
        s_Intake.setDefaultCommand(
            new ManualIntakePivot(
                s_Intake,
                () -> weapons.getRawAxis(XboxController.Axis.kLeftY.value)
            )
        );

        registerStatusSendables();
        configureButtonBindings();
    }

    private void registerNamedCommands() {
        double minPivotAngle = Math.min(Constants.Intake.pivotDownAngleDegrees, Constants.Intake.pivotUpAngleDegrees);
        double maxPivotAngle = Math.max(Constants.Intake.pivotDownAngleDegrees, Constants.Intake.pivotUpAngleDegrees);
        double midPivotAngle = MathUtil.clamp(
            Constants.Intake.pivotDownAngleDegrees + Constants.Intake.pivotMidOffsetDegrees,
            minPivotAngle,
            maxPivotAngle
        );
        NamedCommands.registerCommand("Load", new Load(s_Loader));
        NamedCommands.registerCommand("Shoot", new Shoot(s_Shooter));
        NamedCommands.registerCommand("Run Intake Rollers", new RunIntakeRollers(s_Intake));
        NamedCommands.registerCommand("Lower Intake", new MoveIntakePivotTo(s_Intake, Constants.Intake.pivotDownAngleDegrees));
        NamedCommands.registerCommand("Raise Intake", new MoveIntakePivotTo(s_Intake, Constants.Intake.pivotUpAngleDegrees));
        NamedCommands.registerCommand("Hub Auto Align", new VisionAutoAlign(s_Swerve));
        NamedCommands.registerCommand("Partial Intake", new MoveIntakePivotTo(s_Intake, midPivotAngle));
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

        hubAutoAlign.onTrue(new VisionAutoAlign(s_Swerve, hubAutoAlign::getAsBoolean));
        intakePivotUp.whileTrue(new MoveIntakePivotTo(s_Intake, Constants.Intake.pivotUpAngleDegrees));
        intakePivotDown.whileTrue(new MoveIntakePivotTo(s_Intake, Constants.Intake.pivotDownAngleDegrees));
        double minPivotAngle = Math.min(Constants.Intake.pivotDownAngleDegrees, Constants.Intake.pivotUpAngleDegrees);
        double maxPivotAngle = Math.max(Constants.Intake.pivotDownAngleDegrees, Constants.Intake.pivotUpAngleDegrees);
        double midPivotAngle = MathUtil.clamp(
            Constants.Intake.pivotDownAngleDegrees + Constants.Intake.pivotMidOffsetDegrees,
            minPivotAngle,
            maxPivotAngle
        );
        intakePivotMid.whileTrue(new MoveIntakePivotTo(s_Intake, midPivotAngle));
        intakeRollers.whileTrue(new RunIntakeRollers(s_Intake));
        shoot.whileTrue(new Shoot(s_Shooter));
        load.whileTrue(new Load(s_Loader));
        load.onFalse(Commands.startEnd(s_Loader::runReverse, s_Loader::stop, s_Loader)
            .withTimeout(Constants.Loader.reverseSeconds));

        elasticHubAutoAlign.onTrue(new VisionAutoAlign(s_Swerve, elasticHubAutoAlign::getAsBoolean));
    }

    public Command getAutonomousCommand() {
        s_Swerve.setSpeedMultiplier(1);
        String selectedAuto = elasticSelectedAutoEntry.get();
        if (selectedAuto == null || selectedAuto.isEmpty()) {
            selectedAuto = autoChooser.getSelected();
        }
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
}
