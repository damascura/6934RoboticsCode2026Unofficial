package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.VisionInfo;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;

public class VisionAutoAlign extends Command {
    private static int activeCommandCount = 0;
    private static final double yawToleranceDegrees = 4.0;

    private final Swerve s_Swerve;
    private final BooleanSupplier keepRunning;
    private boolean withinYawTolerance = false;

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
        withinYawTolerance = false;
    }

    @Override
    public void execute() {
        if (!keepRunning.getAsBoolean()) {
            stopDrive();
            return;
        }

        Pose2d pose = s_Swerve.getSwervePoseEstimation();
        Translation2d hubPosition = getAllianceHubPosition();
        Translation2d toHub = hubPosition.minus(pose.getTranslation());
        if (toHub.getNorm() < 1e-6) {
            stopDrive();
            return;
        }

        double desiredYaw = Math.atan2(toHub.getY(), toHub.getX());
        double yawError = MathUtil.angleModulus(desiredYaw - pose.getRotation().getRadians());
        withinYawTolerance = Math.abs(Math.toDegrees(yawError)) <= yawToleranceDegrees;
        if (withinYawTolerance) {
            stopDrive();
            return;
        }
        double yawRate = yawError * Vision.autoAlignYawkP;
        double maxYawRate = Constants.Swerve.maxAngularVelocity * Vision.autoAlignMaxYawPercent;
        yawRate = MathUtil.clamp(yawRate, -maxYawRate, maxYawRate);

        s_Swerve.drive(new Translation2d(), yawRate, true, true);
    }

    @Override
    public void end(boolean interrupted) {
        activeCommandCount = Math.max(0, activeCommandCount - 1);
        stopDrive();
    }

    @Override
    public boolean isFinished() {
        return !keepRunning.getAsBoolean() || withinYawTolerance;
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
}
