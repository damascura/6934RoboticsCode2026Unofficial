package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Intake;

public class ManualIntakePivot extends Command {
    private final Intake intake;
    private final DoubleSupplier percentSupplier;

    public ManualIntakePivot(Intake intake, DoubleSupplier percentSupplier) {
        this.intake = intake;
        this.percentSupplier = percentSupplier;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setManualPercent(percentSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.setManualPercent(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
