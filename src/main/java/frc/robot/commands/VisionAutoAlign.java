package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.VisionInfo;
import frc.robot.subsystems.Swerve;

public class VisionAutoAlign extends Command {
    private static int activeCommandCount = 0;

    private final Swerve s_Swerve;
    private final double targetOffsetXMeters;

    private final PIDController forwardController = new PIDController(
        Vision.autoAlignForwardkP,
        Vision.autoAlignForwardkI,
        Vision.autoAlignForwardkD
    );
    private final PIDController strafeController = new PIDController(
        Vision.autoAlignStrafekP,
        Vision.autoAlignStrafekI,
        Vision.autoAlignStrafekD
    );
    private final PIDController yawController = new PIDController(
        Vision.autoAlignYawkP,
        Vision.autoAlignYawkI,
        Vision.autoAlignYawkD
    );

    public VisionAutoAlign(Swerve s_Swerve, double targetOffsetXMeters) {
        this.s_Swerve = s_Swerve;
        this.targetOffsetXMeters = targetOffsetXMeters;
        addRequirements(s_Swerve);
    }

    public static boolean isAnyAutoAlignActive() {
        return activeCommandCount > 0;
    }

    @Override
    public void initialize() {
        activeCommandCount++;
        VisionInfo.setAprilTagPipeline();
        VisionInfo.setFiducialOffsetX(targetOffsetXMeters);

        forwardController.setTolerance(Vision.autoAlignDistanceToleranceMeters);
        strafeController.setTolerance(Vision.TXTolerance);
        yawController.setTolerance(Vision.autoAlignYawToleranceDegrees);

        forwardController.reset();
        strafeController.reset();
        yawController.reset();
    }

    @Override
    public void execute() {
        double forwardPercent = 0;
        double strafePercent = 0;
        double yawPercent = 0;

        if (VisionInfo.willTarget()) {
            strafePercent = strafeController.calculate(VisionInfo.getTX(false), 0) * Vision.autoAlignStrafeDirection;
            yawPercent = yawController.calculate(VisionInfo.getTagYawErrorDegrees(), 0) * Vision.autoAlignYawDirection;

            double tagDistance = VisionInfo.getNearestTagDistanceMeters();
            if (!Double.isNaN(tagDistance)) {
                forwardPercent = forwardController.calculate(tagDistance, Vision.autoAlignGoalDistanceMeters) * Vision.autoAlignForwardDirection;
            }
        }

        double speedScale = Math.max(s_Swerve.getSpeedMultiplier(), 0.01);
        forwardPercent /= speedScale;
        strafePercent /= speedScale;
        yawPercent /= speedScale;

        forwardPercent = MathUtil.clamp(forwardPercent, -Vision.autoAlignMaxForwardPercent, Vision.autoAlignMaxForwardPercent);
        strafePercent = MathUtil.clamp(strafePercent, -Vision.autoAlignMaxStrafePercent, Vision.autoAlignMaxStrafePercent);
        yawPercent = MathUtil.clamp(yawPercent, -Vision.autoAlignMaxYawPercent, Vision.autoAlignMaxYawPercent);

        s_Swerve.drive(
            new Translation2d(forwardPercent, strafePercent).times(Constants.Swerve.maxSpeed),
            yawPercent * Constants.Swerve.maxAngularVelocity,
            false,
            true
        );
    }

    @Override
    public void end(boolean interrupted) {
        activeCommandCount = Math.max(0, activeCommandCount - 1);
        VisionInfo.resetFiducialOffset();
        s_Swerve.drive(new Translation2d(0, 0), 0, false, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
