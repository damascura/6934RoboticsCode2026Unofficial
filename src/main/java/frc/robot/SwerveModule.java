package frc.robot;

import frc.robot.Constants.Swerve;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber; // The ID number of the swerve module (FL = 0, FR = 1, BL = 2, BR = 3)
    private Rotation2d angleOffset; // Measured for each absolute encoder to set the swerve modules' "zero" rotation (CANCoder)

    private TalonFX mAngleMotor; // Rotates the wheel of the swerve module like a compass ("azimuth" motor)
    private TalonFX mDriveMotor; // Spins the wheel of the swerve module (makes the robot chassis actually move!)
    private CANcoder angleEncoder; // The absolute encoder that constantly measures swerve module rotation

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* Drive motor control (output) requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* Angle (azimuth) motor control (output) requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    /**
     * Creates a SwerveModule object that represents a physical swerve module
     * @param moduleNumber The ID number of the swerve module (FL = 0, FR = 1, BL = 2, BR = 3)
     * @param moduleConstants A SwerveModuleConstants object that contains the swerve module motor IDs, absolute encoder ID, and 
     * absolute encoder angle offset
     * @return A new SwerveModule object
     */
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, Swerve.canivoreName);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, Swerve.canivoreName);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, Swerve.canivoreName);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
    }

    /**
     * Sets the speeds (voltages) of the angle (azimuth) and drive motors of the swerve module
     * @param desiredState A SwerveModuleState object representing the desired angular position of the angle (azimuth) motor and 
     * desired speed of the drive motor for the swerve module
     * @param isOpenLoop If open loop mode should be used. Use open loop mode for teleoperated swerve drive and closed loop mode (having 
     * isOpenLoop = false) for autonomous swerve drive
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState.optimize(getState().angle); // Minimizes movement of the angle motor
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations())); // Set the voltage of the angle motor
        setSpeed(desiredState, isOpenLoop); // Used to set the voltage of the drive motor
    }

    /**
     * Sets the speed (voltage) of the drive motor of the swerve module
     * @param desiredState A SwerveModuleState object representing the desired angular position of the angle (azimuth) motor and 
     * desired speed of the drive motor for the swerve module
     * @param isOpenLoop If open loop mode should be used. Use open loop mode for teleoperated swerve drive and closed loop mode (having 
     * isOpenLoop = false) for autonomous swerve drive
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle); // Set the voltage of the drive motor
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity); // Set the voltage of the drive motor
        }
    }

    /**
     * Gets the absolute angular position of the swerve module using the CANCoder
     * @return A Rotation2d object representing the absolute angular position of the swerve module
     */
    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * Corrects the absolute angular position measured by the CANCoder by subtracting the CANCoder's angular offset from the former. 
     * Assuming the CANCoder's offsets have been measured correctly, this method ensures the swerve modules' "zero" angular 
     * positions are the same
     */
    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    /**
     * Gets the current angular position of the angle (azimuth) motor and current speed of the drive motor for the swerve module
     * @return A SwerveModuleState object containing the current angular position of the angle (azimuth) motor and current speed 
     * of the drive motor for the swerve module 
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }

    /**
     * Gets the current positions of the angle (azimuth) motor and the drive motor for the swerve module
     * @return A SwerveModulePosition object containing the current positions of the angle (azimuth) motor 
     * and the drive motor for the swerve module
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }
}