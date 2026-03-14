package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.VisionInfo;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;

public class VisionAutoAlign extends Command {
    private static int activeCommandCount = 0;

    private final Swerve s_Swerve;
    private final BooleanSupplier keepRunning;
    private Command pathCommand;
    private Pose2d lastTargetPose;
    private double lastReplanTimestamp;

    public VisionAutoAlign(Swerve s_Swerve) {
        this(s_Swerve, () -> true);
    }

    public VisionAutoAlign(Swerve s_Swerve, BooleanSupplier keepRunning) {
        this.s_Swerve = s_Swerve;
        this.keepRunning = keepRunning;
    }

    public static boolean isAnyAutoAlignActive() {
        return activeCommandCount > 0;
    }

    @Override
    public void initialize() {
        activeCommandCount++;
        VisionInfo.setAprilTagPipeline();
        lastTargetPose = null;
        lastReplanTimestamp = Timer.getFPGATimestamp() - Vision.autoAlignReplanPeriodSeconds;
        schedulePathToTarget();
    }

    @Override
    public void execute() {
        if (!keepRunning.getAsBoolean()) {
            cancelPath();
            return;
        }

        if (shouldReplan()) {
            schedulePathToTarget();
        }
    }

    @Override
    public void end(boolean interrupted) {
        activeCommandCount = Math.max(0, activeCommandCount - 1);
        cancelPath();
    }

    @Override
    public boolean isFinished() {
        return !keepRunning.getAsBoolean();
    }

    private Translation2d getAllianceHubPosition() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return new Translation2d(Vision.hubAlignRedX, Vision.hubAlignRedY);
        }
        return new Translation2d(Vision.hubAlignBlueX, Vision.hubAlignBlueY);
    }

    private Pose2d getHubAlignTargetPose() {
        Pose2d pose = s_Swerve.getSwervePoseEstimation();
        Translation2d hubPosition = getAllianceHubPosition();
        Translation2d toHub = hubPosition.minus(pose.getTranslation());
        double distanceToHub = toHub.getNorm();
        if (distanceToHub < 1e-6) {
            return new Pose2d(pose.getTranslation(), pose.getRotation());
        }

        Translation2d direction = toHub.times(1.0 / distanceToHub);
        Translation2d targetTranslation = hubPosition.minus(direction.times(Vision.autoAlignGoalDistanceMeters));
        Rotation2d targetRotation = Rotation2d.fromRadians(Math.atan2(toHub.getY(), toHub.getX()));
        return new Pose2d(targetTranslation, targetRotation);
    }

    private void schedulePathToTarget() {
        Pose2d targetPose = getHubAlignTargetPose();
        PathConstraints constraints = new PathConstraints(
            Constants.Swerve.maxSpeed * Vision.autoAlignMaxForwardPercent,
            Constants.Swerve.maxSpeed * Vision.autoAlignMaxForwardPercent,
            Constants.Swerve.maxAngularVelocity * Vision.autoAlignMaxYawPercent,
            Constants.Swerve.maxAngularVelocity * Vision.autoAlignMaxYawPercent
        );
        cancelPath();
        pathCommand = AutoBuilder.pathfindToPose(targetPose, constraints);
        CommandScheduler.getInstance().schedule(pathCommand);
        lastTargetPose = targetPose;
        lastReplanTimestamp = Timer.getFPGATimestamp();
    }

    private boolean shouldReplan() {
        if (lastTargetPose == null) {
            return true;
        }
        double now = Timer.getFPGATimestamp();
        if (now - lastReplanTimestamp < Vision.autoAlignReplanPeriodSeconds) {
            return false;
        }

        Pose2d targetPose = getHubAlignTargetPose();
        double distanceDelta = targetPose.getTranslation().getDistance(lastTargetPose.getTranslation());
        double rotationDelta = Math.abs(targetPose.getRotation().minus(lastTargetPose.getRotation()).getDegrees());
        return distanceDelta >= Vision.autoAlignReplanTranslationToleranceMeters
            || rotationDelta >= Vision.autoAlignReplanRotationToleranceDegrees;
    }

    private void cancelPath() {
        if (pathCommand != null) {
            CommandScheduler.getInstance().cancel(pathCommand);
            pathCommand = null;
        }
    }
}
