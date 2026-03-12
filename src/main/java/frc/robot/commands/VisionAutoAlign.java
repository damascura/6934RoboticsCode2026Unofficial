package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Vision;
import frc.robot.VisionInfo;
import frc.robot.subsystems.Swerve;

public class VisionAutoAlign extends Command {
    private static int activeCommandCount = 0;

    private final Swerve s_Swerve;
    private boolean hasInitTarget = false;

    public VisionAutoAlign(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    public static boolean isAnyAutoAlignActive() {
        return activeCommandCount > 0;
    }

    @Override
    public void initialize() {
        activeCommandCount++;
        VisionInfo.setAprilTagPipeline();
        hasInitTarget = VisionInfo.hasAnyFiducialIds(Vision.hubAlignTagIds);
    }

    @Override
    public void execute() {
        if (!hasInitTarget) {
            stopDrive();
            return;
        }

        Pose2d pose = s_Swerve.getSwervePoseEstimation();
        Translation2d hubPosition = getAllianceHubPosition();
        Translation2d toHub = hubPosition.minus(pose.getTranslation());
        double distanceToHub = toHub.getNorm();
        if (distanceToHub < 1e-6) {
            stopDrive();
            return;
        }

        double distanceError = distanceToHub - Vision.hubAlignGoalDistanceMeters;
        double speed = distanceError * Vision.hubAlignkP;
        speed = enforceMinimumOutput(
            speed,
            distanceError,
            Vision.hubAlignDistanceToleranceMeters,
            Vision.hubAlignMinSpeedMetersPerSecond
        );
        speed = MathUtil.clamp(speed, -Vision.hubAlignMaxSpeedMetersPerSecond, Vision.hubAlignMaxSpeedMetersPerSecond);

        Translation2d direction = toHub.times(1.0 / distanceToHub);
        Translation2d translation = direction.times(speed);

        double desiredYaw = Math.atan2(toHub.getY(), toHub.getX());
        double yawError = MathUtil.angleModulus(desiredYaw - pose.getRotation().getRadians());
        double yawRate = MathUtil.clamp(
            yawError * Vision.hubAlignYawkP,
            -Vision.hubAlignMaxYawRadiansPerSecond,
            Vision.hubAlignMaxYawRadiansPerSecond
        );

        double speedScale = Math.max(s_Swerve.getSpeedMultiplier(), 0.01);
        Translation2d scaledTranslation = translation.times(1.0 / speedScale);
        s_Swerve.drive(scaledTranslation, yawRate / speedScale, true, true);
    }

    @Override
    public void end(boolean interrupted) {
        activeCommandCount = Math.max(0, activeCommandCount - 1);
        stopDrive();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private Translation2d getAllianceHubPosition() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return new Translation2d(Vision.hubAlignRedX, Vision.hubAlignRedY);
        }
        return new Translation2d(Vision.hubAlignBlueX, Vision.hubAlignBlueY);
    }

    private void stopDrive() {
        s_Swerve.drive(new Translation2d(), 0.0, true, true);
    }

    private double enforceMinimumOutput(double output, double error, double tolerance, double minMagnitude) {
        if (Math.abs(error) <= tolerance || Math.abs(output) < 1e-6) {
            return output;
        }
        return Math.copySign(Math.max(Math.abs(output), minMagnitude), output);
    }
}
