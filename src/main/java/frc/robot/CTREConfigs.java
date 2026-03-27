package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    public TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    public TalonFXConfiguration shooterLeaderConfig = new TalonFXConfiguration();
    public TalonFXConfiguration shooterFollowerConfig = new TalonFXConfiguration();
    public TalonFXConfiguration loaderFeederConfig = new TalonFXConfiguration();
    public TalonFXConfiguration loaderConveyorConfig = new TalonFXConfiguration();
    public TalonFXConfiguration intakePivotConfig = new TalonFXConfiguration();
    public TalonFXConfiguration intakeRollerConfig = new TalonFXConfiguration();
    public CANcoderConfiguration intakePivotCANcoderConfig = new CANcoderConfiguration();


    public CTREConfigs(){

        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;

        /** Shooter Motor Configurations */
        shooterLeaderConfig.MotorOutput.NeutralMode = Constants.Shooter.neutralMode;
        shooterFollowerConfig.MotorOutput.NeutralMode = Constants.Shooter.neutralMode;
        shooterLeaderConfig.MotorOutput.Inverted = Constants.Shooter.motorInvert;
        shooterFollowerConfig.MotorOutput.Inverted = Constants.Shooter.motorInvert;

        shooterLeaderConfig.Slot0.kP = Constants.Shooter.kP;
        shooterLeaderConfig.Slot0.kI = Constants.Shooter.kI;
        shooterLeaderConfig.Slot0.kD = Constants.Shooter.kD;
        shooterLeaderConfig.Slot0.kV = Constants.Shooter.kV;

        /** Loader Motor Configurations */
        loaderFeederConfig.MotorOutput.NeutralMode = Constants.Loader.neutralMode;
        loaderConveyorConfig.MotorOutput.NeutralMode = Constants.Loader.neutralMode;
        loaderFeederConfig.MotorOutput.Inverted = Constants.Loader.feederInvert;
        loaderConveyorConfig.MotorOutput.Inverted = Constants.Loader.conveyorInvert;

        /** Intake Motor Configurations */
        intakePivotConfig.MotorOutput.NeutralMode = Constants.Intake.neutralMode;
        intakePivotConfig.MotorOutput.Inverted = Constants.Intake.pivotInvert;

        intakeRollerConfig.MotorOutput.NeutralMode = Constants.Intake.neutralMode;
        intakeRollerConfig.MotorOutput.Inverted = Constants.Intake.rollerInvert;

        /** Intake Pivot CANcoder Configuration */
        intakePivotCANcoderConfig.MagnetSensor.SensorDirection = Constants.Intake.pivotCanCoderDirection;
    }
}
