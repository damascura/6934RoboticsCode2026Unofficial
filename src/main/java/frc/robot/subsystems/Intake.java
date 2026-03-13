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
    private double manualPercent = 0.0;
    private boolean profileEnabled = false;

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

    public void startProfileToggle() {
        double currentDegrees = getPivotAngleDegrees();
        double midDegrees = (Constants.Intake.pivotUpAngleDegrees + Constants.Intake.pivotDownAngleDegrees) / 2.0;
        double targetDegrees = currentDegrees > midDegrees
            ? Constants.Intake.pivotDownAngleDegrees
            : Constants.Intake.pivotUpAngleDegrees;

        startProfileTo(targetDegrees);
    }

    public void startProfileTo(double targetDegrees) {
        profileEnabled = true;
        pivotGoalDegrees = targetDegrees;
        pivotController.reset(getPivotAngleDegrees());
        pivotController.setGoal(pivotGoalDegrees);
    }

    public void setPivotGoalDegrees(double goalDegrees) {
        pivotGoalDegrees = goalDegrees;
        pivotController.setGoal(pivotGoalDegrees);
    }

    public void disableProfileHold() {
        profileEnabled = false;
        double currentDegrees = getPivotAngleDegrees();
        pivotGoalDegrees = currentDegrees;
        pivotController.reset(currentDegrees);
        pivotController.setGoal(currentDegrees);
    }

    public void setManualPercent(double percent) {
        manualPercent = percent;
    }

    public boolean isDown() {
        return Math.abs(getPivotAngleDegrees() - Constants.Intake.pivotDownAngleDegrees)
            <= Constants.Intake.pivotToleranceDegrees;
    }

    public boolean isGoalDown() {
        return Math.abs(pivotGoalDegrees - Constants.Intake.pivotDownAngleDegrees)
            <= Constants.Intake.pivotToleranceDegrees;
    }

    public boolean isAtGoal() {
        return pivotController.atGoal();
    }

    public boolean isManualActive() {
        return Math.abs(applyDeadband(manualPercent, Constants.Intake.pivotManualDeadband)) > 1e-6;
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
        double manualInput = applyDeadband(manualPercent, Constants.Intake.pivotManualDeadband);
        if (Math.abs(manualInput) > 1e-6) {
            double currentDegrees = getPivotAngleDegrees();
            double requestedVolts = manualInput * Constants.Intake.pivotManualMaxVoltage;
            double minDegrees = Math.min(Constants.Intake.pivotUpAngleDegrees, Constants.Intake.pivotDownAngleDegrees);
            double maxDegrees = Math.max(Constants.Intake.pivotUpAngleDegrees, Constants.Intake.pivotDownAngleDegrees);

            if ((requestedVolts > 0 && currentDegrees >= maxDegrees)
                || (requestedVolts < 0 && currentDegrees <= minDegrees)) {
                requestedVolts = 0.0;
            }

            profileEnabled = false;
            pivotGoalDegrees = currentDegrees;
            pivotController.reset(currentDegrees);
            pivotController.setGoal(currentDegrees);
            pivotMotor.setControl(pivotVoltageRequest.withOutput(requestedVolts));
            return;
        }

        double outputVolts = pivotController.calculate(getPivotAngleDegrees());
        double maxVoltage = profileEnabled
            ? Constants.Intake.pivotMaxVoltage
            : Constants.Intake.pivotHoldMaxVoltage;
        outputVolts = Math.max(-maxVoltage, Math.min(maxVoltage, outputVolts));
        pivotMotor.setControl(pivotVoltageRequest.withOutput(outputVolts));
    }

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) <= deadband) {
            return 0.0;
        }
        return value;
    }

    private double degreesToPivotRotations(double degrees) {
        return degrees / 360.0;
    }
}
