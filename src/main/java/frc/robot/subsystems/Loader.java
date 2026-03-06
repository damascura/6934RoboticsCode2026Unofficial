package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Loader extends SubsystemBase {
    private final TalonFX feederMotor;
    private final TalonFX conveyorMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Loader() {
        feederMotor = new TalonFX(Constants.Loader.feederMotorID, Constants.Loader.canBusRef);
        conveyorMotor = new TalonFX(Constants.Loader.conveyorMotorID, Constants.Loader.canBusRef);

        feederMotor.getConfigurator().apply(Robot.ctreConfigs.loaderFeederConfig);
        conveyorMotor.getConfigurator().apply(Robot.ctreConfigs.loaderConveyorConfig);
    }

    public void runForward() {
        feederMotor.setControl(voltageRequest.withOutput(Constants.Loader.feederForwardVoltage));
        conveyorMotor.setControl(voltageRequest.withOutput(Constants.Loader.conveyorForwardVoltage));
    }

    public void runReverse() {
        feederMotor.setControl(voltageRequest.withOutput(Constants.Loader.feederReverseVoltage));
        conveyorMotor.setControl(voltageRequest.withOutput(Constants.Loader.conveyorReverseVoltage));
    }

    public void stop() {
        feederMotor.stopMotor();
        conveyorMotor.stopMotor();
    }
}
