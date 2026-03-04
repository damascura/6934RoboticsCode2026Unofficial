package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsys;

public class Shoot extends Command {
    private final ShooterSubsys s_Shooter;

    public Shoot(ShooterSubsys s_Shooter) {
        this.s_Shooter = s_Shooter;
        addRequirements(s_Shooter);
    }

    @Override
    public void execute() {
        s_Shooter.runAtTargetRPS();
    }

    @Override
    public void end(boolean interrupted) {
        s_Shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
