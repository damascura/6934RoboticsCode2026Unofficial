package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.VisionInfo;
import frc.robot.subsystems.Swerve;

public class CorrectHorizontalError extends Command {
    private Swerve s_Swerve;
    private ProfiledPIDController horizontalPIDController;
    private double goalTX;

    public CorrectHorizontalError(Swerve s_Swerve, double goalTX) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        horizontalPIDController = new ProfiledPIDController(3, 
                                                            Vision.TXkI, 
                                                            Vision.TXkD, 
                                                            new TrapezoidProfile.Constraints(Vision.TXMaxSpeed, Vision.TXMaxAcceleration));

        this.goalTX = goalTX;
    }

    @Override
    public void initialize() {
        horizontalPIDController.setTolerance(Units.degreesToRotations(Vision.TXTolerance));
        horizontalPIDController.reset(Units.degreesToRotations(VisionInfo.getTX(false)));
    }

    @Override
    public void execute() {
        double horizontalOutput = 0;
        if (VisionInfo.willTarget()) {
            horizontalOutput = horizontalPIDController.calculate(Units.degreesToRotations(VisionInfo.getTX(false)), goalTX) / s_Swerve.getSpeedMultiplier();
        }
        
        s_Swerve.drive( // Drive
            new Translation2d(0, horizontalOutput).times(Constants.Swerve.maxSpeed), 
            0, 
            false, 
            true
        );
    }

    @Override
    public boolean isFinished() { // Ensures an infinite loop does not occur if the camera loses track of the AprilTag
        return (horizontalPIDController.atGoal() || !VisionInfo.willTarget());
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive( // Stops the swerve drive
            new Translation2d(0, 0), 
            0, 
            false, 
            true
        );
    }
}
