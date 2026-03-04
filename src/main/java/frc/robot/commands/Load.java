package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Loader;

public class Load extends Command {
    private final Loader s_Loader;

    public Load(Loader s_Loader) {
        this.s_Loader = s_Loader;
        addRequirements(s_Loader);
    }

    @Override
    public void execute() {
        s_Loader.runForward();
    }

    @Override
    public void end(boolean interrupted) {
        s_Loader.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
