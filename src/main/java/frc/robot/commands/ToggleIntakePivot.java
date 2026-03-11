package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class ToggleIntakePivot extends InstantCommand {
    public ToggleIntakePivot(Intake intake) {
        super(intake::togglePivot);
    }
}
