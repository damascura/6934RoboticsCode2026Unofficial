package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public final class Constants {

    // #region QuickTuning
    public static final class QuickTuning {
        /* Controller Constants */
        public static final int driveControllerID = 0;
        public static final int weaponControllerID = 1;

        public static final double driveStickDeadband = 0.1;
        public static final double weaponStickDeadband = 0.1;

        public static final double driveSlowModeMultiplier = 0.1;

        /* Robot Starting Position */
        public static final Pose2d selectedStartingPose = new Pose2d();
    }
    //#endregion

    public static final class Vision {
        /* Limelight Configs & Location */
        // TODO: set LL pos and rot
        public static final String limelightName = "limelight-scorps";

        public static final double limelightXPos = 0; // Meters, +x is right, -x is left
        public static final double limelightYPos = 0; // Meters, +y is forward, -y is backward
        public static final double limelightZPos = 0; // Meters, +z is up, -z is down

        public static final double limelightRoll = Units.degreesToRadians(0); // Radians, counterclockwise positive, rotation about x-axis
        public static final double limelightPitch = Units.degreesToRadians(0); // Radians, tilted up is positive, down is negative
        public static final double limelightYaw = Units.degreesToRadians(0); // Radians, counterclockwise positive, rotation aobut z-axis

        public static final Transform3d limelightPositionOnBot = new Transform3d(
            new Translation3d(limelightXPos, limelightYPos, limelightZPos),
            new Rotation3d(limelightRoll, limelightPitch, limelightYaw)
        );

        /* Targetting Threshold */
        public static final int targetDetectionListSize = 10; // Amount of trials the list holds
        public static final double averageTVThreshold = 0.7; // Required targetting success rate for automatic alignment

        /* Vision Alignment PID Constants */
        public static final double TXkP = 0.15;
        public static final double TXkI = 0.0;
        public static final double TXkD = 0.0;
        public static final double TXMaxSpeed = 1.0;
        public static final double TXMaxAcceleration = 0.5;

        /* Alignment Error Tolerances */
        public static final double TXTolerance = 1; // Degrees
        public static final double TYTolerance = 1; // Degrees
        public static final double poseTolerance = 1; // Degrees

        /* Auto Align */
        public static final int aprilTagPipeline = 0;

        public static final double autoAlignGoalDistanceMeters = 0.8;
        public static final double autoAlignDistanceToleranceMeters = 0.08;
        public static final double autoAlignYawToleranceDegrees = 3.0;

        public static final double autoAlignForwardkP = 1.2;
        public static final double autoAlignForwardkI = 0.0;
        public static final double autoAlignForwardkD = 0.0;

        public static final double autoAlignStrafekP = 0.05;
        public static final double autoAlignStrafekI = 0.0;
        public static final double autoAlignStrafekD = 0.0;

        public static final double autoAlignYawkP = 0.015;
        public static final double autoAlignYawkI = 0.0;
        public static final double autoAlignYawkD = 0.0;

        public static final double autoAlignMaxForwardPercent = 0.35;
        public static final double autoAlignMaxStrafePercent = 0.35;
        public static final double autoAlignMaxYawPercent = 0.35;

        public static final double autoAlignForwardDirection = -1.0;
        public static final double autoAlignStrafeDirection = 1.0;
        public static final double autoAlignYawDirection = 1.0;

        public static final double autoAlignCenterOffsetX = 0.0;
        public static final double autoAlignLeftOffsetX = -0.16;
        public static final double autoAlignRightOffsetX = 0.16;
        public static final double autoAlignOffsetY = 0.0;
        public static final double autoAlignOffsetZ = 0.0;

        public static final double maxVisionYawRateForMegatag = 540.0;
        public static final int minVisionTagCountForMegatag = 1;
        public static final double maxVisionAvgTagDistMeters = 4.5;
        public static final double minVisionAvgTagArea = 0.02;
        public static final double maxVisionFiducialAmbiguity = 0.7;

        public static final double visionStdDevXYMin = 0.15;
        public static final double visionStdDevXYMax = 2.0;


        /* AprilTag Game Field Layout Loading */
        public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    }

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final String canivoreName = "Second CANivor<3";

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double robotSideLength = Units.inchesToMeters(28);
        public static final double trackWidth = Units.inchesToMeters(22.75); 
        public static final double wheelBase = Units.inchesToMeters(22.875); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; // Originally 0.12
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // TODO: Check later
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5;
        /** Radians per Second */
        public static final double maxAngularVelocity =  3 * Math.PI;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(131.8359375);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(133.68164 + 180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(50.1855); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-73.740234375 + 180); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double maxModuleSpeed = 4.5; // Max module speed, in m/s
        public static final double driveBaseRadius = (Swerve.robotSideLength / 2) * Math.sqrt(2);
        public static final PPHolonomicDriveController pathPlannerConfig = new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0)
        );
    }
}
