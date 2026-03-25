package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.SwerveModule;
import frc.robot.VisionInfo;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator swervePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private double speedMultiplier;
    private Pose2d startingPose;
    private SwerveModuleState[] desiredModuleStates;
    private final Field2d field2d = new Field2d();

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.canBus);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        speedMultiplier = 1;

        // Keep all robot pose math in the WPILib blue-alliance field frame.
        startingPose = Constants.QuickTuning.selectedStartingPose;

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        desiredModuleStates = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };

        VisionInfo.setAprilTagPipeline();
        VisionInfo.resetFiducialOffset();
        registerElasticSwerveWidget();
        SmartDashboard.putData("Field", field2d);

        Timer.delay(1);
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        swervePoseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            startingPose,
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );
        System.out.println("Swerve subsystem loaded!");

        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getSwervePoseEstimation,
                this::setSwervePoseEstimate,
                this::getChassisSpeeds,
                this::driveRobotRelative,
                Constants.AutoConstants.pathPlannerConfig,
                config,
                () -> {
                    var currentAlliance = DriverStation.getAlliance();
                    if (!currentAlliance.isPresent()) {
                        return Constants.AutoConstants.flipAutosWhenAllianceUnknown;
                    }
                    if (currentAlliance.get() == DriverStation.Alliance.Red) {
                        return Constants.AutoConstants.flipAutosOnRedAlliance;
                    }
                    return Constants.AutoConstants.flipAutosOnBlueAlliance;
                },
                this
            );
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void registerElasticSwerveWidget() {
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> mSwerveMods[0].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> mSwerveMods[0].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", () -> mSwerveMods[1].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> mSwerveMods[1].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", () -> mSwerveMods[2].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> mSwerveMods[2].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", () -> mSwerveMods[3].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> mSwerveMods[3].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty(
                    "Robot Angle",
                    () -> (swerveOdometry != null) ? getHeading().getRadians() : getGyroYaw().getRadians(),
                    null
                );
            }
        });
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX() * speedMultiplier,
                        translation.getY() * speedMultiplier,
                        rotation * speedMultiplier,
                        getHeading()
                    )
                    : new ChassisSpeeds(
                        translation.getX() * speedMultiplier,
                        translation.getY() * speedMultiplier,
                        rotation * speedMultiplier
                    )
            );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        desiredModuleStates = copyModuleStates(swerveModuleStates);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void setSpeedMultiplier(double newSpeed) {
        speedMultiplier = newSpeed;
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        drive(
            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
            speeds.omegaRadiansPerSecond,
            false,
            false
        );
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        desiredModuleStates = copyModuleStates(desiredStates);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds fieldRelativeSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getGyroYaw());
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void setSwervePoseEstimate(Pose2d pose) {
        swervePoseEstimator.resetPose(pose);
    }

    public void updateSwervePoseEstimator() {
        swervePoseEstimator.update(getGyroYaw(), getModulePositions());

        double yawDegrees = getGyroYaw().getDegrees();
        double yawRateDegreesPerSecond = gyro.getAngularVelocityZWorld().getValueAsDouble();
        double pitchDegrees = gyro.getPitch().getValueAsDouble();
        double pitchRateDegreesPerSecond = gyro.getAngularVelocityYWorld().getValueAsDouble();
        double rollDegrees = gyro.getRoll().getValueAsDouble();
        double rollRateDegreesPerSecond = gyro.getAngularVelocityXWorld().getValueAsDouble();

        LimelightHelpers.SetRobotOrientation_NoFlush(
            Constants.Vision.limelightName,
            yawDegrees,
            yawRateDegreesPerSecond,
            pitchDegrees,
            pitchRateDegreesPerSecond,
            rollDegrees,
            rollRateDegreesPerSecond
        );

        LimelightHelpers.PoseEstimate poseEstimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.limelightName);

        boolean hasEstimate = (poseEstimate != null) && (poseEstimate.pose != null);
        boolean validTargets = VisionInfo.hasValidTargets();
        boolean spinRateOk = Math.abs(yawRateDegreesPerSecond) < Constants.Vision.maxVisionYawRateForMegatag;
        boolean tagCountOk = hasEstimate && (poseEstimate.tagCount >= Constants.Vision.minVisionTagCountForMegatag);
        boolean avgDistOk = hasEstimate && (poseEstimate.avgTagDist <= Constants.Vision.maxVisionAvgTagDistMeters);
        boolean avgAreaOk = hasEstimate && (poseEstimate.avgTagArea >= Constants.Vision.minVisionAvgTagArea);
        boolean ambiguityOk = hasEstimate && hasAcceptableFiducialAmbiguity(poseEstimate);
        boolean acceptVisionFrame = hasEstimate && validTargets && spinRateOk && tagCountOk && avgDistOk && avgAreaOk && ambiguityOk;

        Logger.recordOutput("Vision/MegaTag1/Accepted", acceptVisionFrame);
        Logger.recordOutput("Vision/MegaTag1/HasEstimate", hasEstimate);
        Logger.recordOutput("Vision/MegaTag1/ValidTargets", validTargets);
        Logger.recordOutput("Vision/MegaTag1/TagCountOk", tagCountOk);
        Logger.recordOutput("Vision/MegaTag1/AvgDistOk", avgDistOk);
        Logger.recordOutput("Vision/MegaTag1/AvgAreaOk", avgAreaOk);
        Logger.recordOutput("Vision/MegaTag1/TagCount", hasEstimate ? poseEstimate.tagCount : -1);
        Logger.recordOutput("Vision/MegaTag1/AvgTagDistMeters", hasEstimate ? poseEstimate.avgTagDist : -1);
        Logger.recordOutput("Vision/MegaTag1/AvgTagArea", hasEstimate ? poseEstimate.avgTagArea : -1);
        Logger.recordOutput("Vision/MegaTag1/SpinRateOk", spinRateOk);
        Logger.recordOutput("Vision/MegaTag1/AmbiguityOk", ambiguityOk);

        if (acceptVisionFrame) {
            double xyStdDev = computeVisionXYStdDev(poseEstimate);
            swervePoseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(Constants.Vision.visionStdDevYawDegrees))
            );
            swervePoseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
            Logger.recordOutput("Vision/MegaTag1/XYStdDev", xyStdDev);
        }
    }

    public Pose2d getSwervePoseEstimation() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    private SwerveModuleState[] copyModuleStates(SwerveModuleState[] states) {
        SwerveModuleState[] copy = new SwerveModuleState[states.length];
        for (int i = 0; i < states.length; i++) {
            copy[i] = new SwerveModuleState(states[i].speedMetersPerSecond, states[i].angle);
        }
        return copy;
    }

    private boolean hasAcceptableFiducialAmbiguity(LimelightHelpers.PoseEstimate poseEstimate) {
        if (poseEstimate.rawFiducials == null || poseEstimate.rawFiducials.length == 0) {
            return true;
        }

        for (LimelightHelpers.RawFiducial fiducial : poseEstimate.rawFiducials) {
            if (fiducial.ambiguity > Constants.Vision.maxVisionFiducialAmbiguity) {
                return false;
            }
        }
        return true;
    }

    private double computeVisionXYStdDev(LimelightHelpers.PoseEstimate poseEstimate) {
        double baseStdDev;
        if (poseEstimate.tagCount >= 2) {
            baseStdDev = 0.20 + (0.08 * poseEstimate.avgTagDist);
        } else {
            baseStdDev = 0.40 + (0.18 * poseEstimate.avgTagDist);
        }

        if (poseEstimate.avgTagArea < 0.08) {
            baseStdDev += 0.20;
        }

        return Math.max(Constants.Vision.visionStdDevXYMin, Math.min(baseStdDev, Constants.Vision.visionStdDevXYMax));
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        updateSwervePoseEstimator();

        VisionInfo.updateDashboardValues(
            swervePoseEstimator.getEstimatedPosition().getX(),
            swervePoseEstimator.getEstimatedPosition().getY(),
            swervePoseEstimator.getEstimatedPosition().getRotation().getDegrees()
        );
        SmartDashboard.putBoolean("Vision Status/Valid AprilTag Detected", VisionInfo.hasValidTargets());

        Logger.recordOutput("Swerve/States/Desired", desiredModuleStates);
        Logger.recordOutput("Swerve/States/Measured", getModuleStates());
        Logger.recordOutput("Swerve/Positions", getModulePositions());
        Logger.recordOutput("Swerve/Pose/Odometry", getPose());
        Pose2d estimatedPose = getSwervePoseEstimation();
        Logger.recordOutput("Swerve/Pose/Estimated", estimatedPose);
        Logger.recordOutput("Swerve/Pose/EstimatedBlue", estimatedPose);
        Logger.recordOutput("Swerve/Gyro/YawDegrees", getGyroYaw().getDegrees());
        field2d.setRobotPose(estimatedPose);

        for (SwerveModule mod : mSwerveMods) {
            Logger.recordOutput("Swerve/Module" + mod.moduleNumber + "/CANcoderDegrees", mod.getCANcoder().getDegrees());
            Logger.recordOutput("Swerve/Module" + mod.moduleNumber + "/AngleDegrees", mod.getPosition().angle.getDegrees());
            Logger.recordOutput("Swerve/Module" + mod.moduleNumber + "/VelocityMetersPerSecond", mod.getState().speedMetersPerSecond);
        }
    }
}
