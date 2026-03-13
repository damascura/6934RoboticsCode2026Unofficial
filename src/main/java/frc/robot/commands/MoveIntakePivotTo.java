package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class MoveIntakePivotTo extends Command {
    private final Intake intake;
    private final double targetDegrees;

    public MoveIntakePivotTo(Intake intake, double targetDegrees) {
        this.intake = intake;
        this.targetDegrees = targetDegrees;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.startProfileTo(targetDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        intake.disableProfileHold();
    }

    @Override
    public boolean isFinished() {
        return intake.isAtGoal() || intake.isManualActive();
    }
}
