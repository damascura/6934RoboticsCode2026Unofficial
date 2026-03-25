package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;

public class VisionAutoAlign extends Command {
    private static int activeCommandCount = 0;

    private final Swerve s_Swerve;
    private final BooleanSupplier keepRunning;
    private final SlewRateLimiter speedLimiter;
    private final SlewRateLimiter yawRateLimiter;
    private double distanceErrorMeters = Double.POSITIVE_INFINITY;
    private double yawErrorRadians = Double.POSITIVE_INFINITY;

    public VisionAutoAlign(Swerve s_Swerve) {
        this(s_Swerve, () -> true);
    }

    public VisionAutoAlign(Swerve s_Swerve, BooleanSupplier keepRunning) {
        this.s_Swerve = s_Swerve;
        this.keepRunning = keepRunning;
        this.speedLimiter = new SlewRateLimiter(Vision.hubAlignMaxAccelerationMetersPerSecondSq);
        this.yawRateLimiter = new SlewRateLimiter(Vision.hubAlignMaxYawAccelerationRadiansPerSecondSq);
        addRequirements(s_Swerve);
    }

    public static boolean isAnyAutoAlignActive() {
        return activeCommandCount > 0;
    }

    @Override
    public void initialize() {
        activeCommandCount++;
        distanceErrorMeters = Double.POSITIVE_INFINITY;
        yawErrorRadians = Double.POSITIVE_INFINITY;
        speedLimiter.reset(0.0);
        yawRateLimiter.reset(0.0);
    }

    @Override
    public void execute() {
        if (!keepRunning.getAsBoolean()) {
            stopDrive();
            return;
        }

        Pose2d robotPose = s_Swerve.getSwervePoseEstimation();
        Translation2d hubPosition = getAllianceHubPosition();
        Translation2d toHub = hubPosition.minus(robotPose.getTranslation());

        double distanceToHub = toHub.getNorm();
        if (distanceToHub < 1e-6) {
            stopDrive();
            return;
        }

        double desiredYaw = Math.atan2(toHub.getY(), toHub.getX());
        yawErrorRadians = MathUtil.angleModulus(desiredYaw - robotPose.getRotation().getRadians());
        double yawTarget = 0.0;
        if (Math.abs(yawErrorRadians) > Math.toRadians(Vision.hubAlignYawToleranceDegrees)) {
            yawTarget = MathUtil.clamp(
                yawErrorRadians * Vision.hubAlignYawkP,
                -Vision.hubAlignMaxYawRadiansPerSecond,
                Vision.hubAlignMaxYawRadiansPerSecond
            );
        }
        double yawCommand = yawRateLimiter.calculate(yawTarget);

        distanceErrorMeters = distanceToHub - Vision.hubAlignGoalDistanceMeters;
        double radialSpeedTarget = 0.0;
        if (Math.abs(distanceErrorMeters) > Vision.hubAlignDistanceToleranceMeters) {
            radialSpeedTarget = MathUtil.clamp(
                distanceErrorMeters * Vision.hubAlignkP,
                -Vision.hubAlignMaxSpeedMetersPerSecond,
                Vision.hubAlignMaxSpeedMetersPerSecond
            );

            // Ensure enough command to overcome stiction when not at goal.
            if (Math.abs(radialSpeedTarget) < Vision.hubAlignMinSpeedMetersPerSecond) {
                radialSpeedTarget = Math.copySign(Vision.hubAlignMinSpeedMetersPerSecond, radialSpeedTarget);
            }
        }
        double radialSpeed = speedLimiter.calculate(radialSpeedTarget);
        double speedMultiplier = Math.max(1e-6, s_Swerve.getSpeedMultiplier());

        Translation2d translationCommand = toHub.div(distanceToHub).times(radialSpeed / speedMultiplier);
        s_Swerve.drive(
            translationCommand,
            yawCommand / speedMultiplier,
            true,
            true
        );
    }

    @Override
    public void end(boolean interrupted) {
        activeCommandCount = Math.max(0, activeCommandCount - 1);
        stopDrive();
    }

    @Override
    public boolean isFinished() {
        return !keepRunning.getAsBoolean();
    }

    private void stopDrive() {
        s_Swerve.drive(new Translation2d(), 0.0, true, true);
    }

    private Translation2d getAllianceHubPosition() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return new Translation2d(Constants.Vision.hubAlignRedX, Constants.Vision.hubAlignRedY);
        }
        return new Translation2d(Constants.Vision.hubAlignBlueX, Constants.Vision.hubAlignBlueY);
    }
}
