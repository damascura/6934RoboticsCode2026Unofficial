package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.Constants.QuickTuning;
import frc.robot.Constants.Vision;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.VisionAutoAlign;
import frc.robot.subsystems.Swerve;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private static final List<String> elasticAutoOptions = List.of("DoNothing");

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

    private final JoystickButton autoAlignCenter = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton autoAlignLeft = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton autoAlignRight = new JoystickButton(driver, XboxController.Button.kB.value);
    private final NetworkButton elasticAutoAlignCenter;
    private final NetworkButton elasticAutoAlignLeft;
    private final NetworkButton elasticAutoAlignRight;

    /* Elastic Topics */
    private final StringArrayPublisher elasticAutoOptionsPublisher;
    private final StringEntry elasticSelectedAutoEntry;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    public RobotContainer() {
        NetworkTable elasticTable = NetworkTableInstance.getDefault().getTable("Elastic");
        elasticAutoOptionsPublisher = elasticTable.getStringArrayTopic("Auto/Options").publish();
        elasticSelectedAutoEntry = elasticTable.getStringTopic("Auto/Selected").getEntry("DoNothing");

        elasticAutoOptionsPublisher.set(elasticAutoOptions.toArray(new String[0]));
        elasticSelectedAutoEntry.set("DoNothing");

        elasticAutoAlignCenter = new NetworkButton(elasticTable, "Commands/AutoAlignCenter");
        elasticAutoAlignLeft = new NetworkButton(elasticTable, "Commands/AutoAlignLeft");
        elasticAutoAlignRight = new NetworkButton(elasticTable, "Commands/AutoAlignRight");

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

    private void registerStatusSendables() {
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
                builder.addBooleanProperty(
                    "Autonomous Or AutoAlign",
                    () -> DriverStation.isAutonomousEnabled() || VisionAutoAlign.isAnyAutoAlignActive(),
                    null
                );
            }
        });
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

        elasticAutoAlignCenter.whileTrue(new VisionAutoAlign(s_Swerve, Vision.autoAlignCenterOffsetX));
        elasticAutoAlignLeft.whileTrue(new VisionAutoAlign(s_Swerve, Vision.autoAlignLeftOffsetX));
        elasticAutoAlignRight.whileTrue(new VisionAutoAlign(s_Swerve, Vision.autoAlignRightOffsetX));
    }

    public Command getAutonomousCommand() {
        s_Swerve.setSpeedMultiplier(1);
        String selectedAuto = elasticSelectedAutoEntry.get("DoNothing");
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
