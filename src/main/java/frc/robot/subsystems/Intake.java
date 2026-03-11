package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
    private final TalonFX pivotMotor;
    private final TalonFX rollerMotor;
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);

    private final ProfiledPIDController pivotController = new ProfiledPIDController(
        Constants.Intake.pivotkP,
        Constants.Intake.pivotkI,
        Constants.Intake.pivotkD,
        new TrapezoidProfile.Constraints(
            Constants.Intake.pivotProfileMaxVelocityDegPerSec,
            Constants.Intake.pivotProfileMaxAccelerationDegPerSecSq
        )
    );

    private double pivotGoalDegrees = Constants.Intake.pivotUpAngleDegrees;

    public Intake() {
        pivotMotor = new TalonFX(Constants.Intake.pivotMotorID, Constants.Intake.canBusRef);
        rollerMotor = new TalonFX(Constants.Intake.rollerMotorID, Constants.Intake.canBusRef);

        pivotMotor.getConfigurator().apply(Robot.ctreConfigs.intakePivotConfig);
        rollerMotor.getConfigurator().apply(Robot.ctreConfigs.intakeRollerConfig);

        // Assume we start at the up position on boot
        pivotMotor.setPosition(degreesToPivotRotations(Constants.Intake.pivotUpAngleDegrees));

        pivotController.setTolerance(Constants.Intake.pivotToleranceDegrees);
        pivotController.reset(getPivotAngleDegrees());
        pivotController.setGoal(pivotGoalDegrees);
    }

    public void togglePivot() {
        if (isGoalDown()) {
            setPivotGoalDegrees(Constants.Intake.pivotUpAngleDegrees);
            return;
        }
        setPivotGoalDegrees(Constants.Intake.pivotDownAngleDegrees);
    }

    public void setPivotGoalDegrees(double goalDegrees) {
        pivotGoalDegrees = goalDegrees;
        pivotController.setGoal(pivotGoalDegrees);
    }

    public boolean isDown() {
        return Math.abs(getPivotAngleDegrees() - Constants.Intake.pivotDownAngleDegrees)
            <= Constants.Intake.pivotToleranceDegrees;
    }

    public boolean isGoalDown() {
        return Math.abs(pivotGoalDegrees - Constants.Intake.pivotDownAngleDegrees)
            <= Constants.Intake.pivotToleranceDegrees;
    }

    public double getPivotAngleDegrees() {
        return pivotMotor.getPosition().getValueAsDouble() * 360.0;
    }

    public void runRollers() {
        rollerMotor.setControl(rollerVoltageRequest.withOutput(Constants.Intake.rollerVoltage));
    }

    public void reverseRollers() {
        rollerMotor.setControl(rollerVoltageRequest.withOutput(Constants.Intake.rollerReverseVoltage));
    }

    public void stopRollers() {
        rollerMotor.stopMotor();
    }

    @Override
    public void periodic() {
        double outputVolts = pivotController.calculate(getPivotAngleDegrees());
        double maxVoltage = Math.abs(pivotController.getPositionError()) <= Constants.Intake.pivotToleranceDegrees
            ? Constants.Intake.pivotHoldMaxVoltage
            : Constants.Intake.pivotMaxVoltage;
        outputVolts = Math.max(-maxVoltage, Math.min(maxVoltage, outputVolts));
        pivotMotor.setControl(pivotVoltageRequest.withOutput(outputVolts));
    }

    private double degreesToPivotRotations(double degrees) {
        return degrees / 360.0;
    }
}
