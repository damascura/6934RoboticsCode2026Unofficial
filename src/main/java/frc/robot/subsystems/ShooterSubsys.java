package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsys extends SubsystemBase {
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public ShooterSubsys() {
        leaderMotor = new TalonFX(Constants.Shooter.leaderMotorID, Constants.Shooter.canBus);
        followerMotor = new TalonFX(Constants.Shooter.followerMotorID, Constants.Shooter.canBus);

        leaderMotor.getConfigurator().apply(Robot.ctreConfigs.shooterLeaderConfig);
        followerMotor.getConfigurator().apply(Robot.ctreConfigs.shooterFollowerConfig);

        MotorAlignmentValue alignment = Constants.Shooter.followerOpposesLeader
            ? MotorAlignmentValue.Opposed
            : MotorAlignmentValue.Aligned;
        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), alignment));
    }

    public void runAtTargetRPS() {
        leaderMotor.setControl(velocityRequest.withVelocity(Constants.Shooter.targetRPS));
    }

    public void stop() {
        leaderMotor.stopMotor();
    }

    public double getLeaderRPS() {
        return leaderMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/TargetRPS", Constants.Shooter.targetRPS);
        Logger.recordOutput("Shooter/LeaderRPS", getLeaderRPS());
    }
}
