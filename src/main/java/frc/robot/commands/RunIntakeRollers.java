package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntakeRollers extends Command {
    private final Intake intake;

    public RunIntakeRollers(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.runRollers();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
