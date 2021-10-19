// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    /** Holds constants like the angle calibration. */
    private final SwerveConstants constants;

    /** The motor controlling the angle of the swerve module. */
    private final TalonFX angleMotor;

    /** The motor controlling the speed of the swerve module. */
    private final TalonFX driveMotor;

    private PIDController steeringControl = new PIDController(0.1, 0, 0);

    public SwerveModule(SwerveConstants constants) {
        this.angleMotor = new TalonFX(constants.angleMotor);
        this.driveMotor = new TalonFX(constants.driveMotor);
        this.constants = constants;

        driveMotor.configFactoryDefault();
        angleMotor.configFactoryDefault();

        driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 60, 0.03));
        angleMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 60, 0.03));

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms);
        angleMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms);
        driveMotor.configVelocityMeasurementWindow(4);
        angleMotor.configVelocityMeasurementWindow(4);

        angleMotor.setInverted(true);

        driveMotor.configVoltageCompSaturation(12);
        angleMotor.configVoltageCompSaturation(12);
        driveMotor.enableVoltageCompensation(true);
        angleMotor.enableVoltageCompensation(true);

        driveMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setNeutralMode(NeutralMode.Brake);

        
    }

    /** Drives the swerve module in a direction at a speed.
     * 
     * @param angle The direction to point the swerve module.
     * @param speed The speed of the drive motor. [Range 0..1]
     */
    public void drive(double angle, double speed) {
        driveMotor.set(ControlMode.Velocity, speed);
        double currentAngle = angleMotor.getSelectedSensorPosition() / 72.81;
        currentAngle = currentAngle % 360;
        if (angle - currentAngle > 180){
            angle -= 360;
        }
        angleMotor.set(ControlMode.Position, angle * 72.81);
    }

    /**
     * Get the current state of a swerve module
     * @return SwerveModuleState representing the current speed and angle of the module
     */
    
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            driveMotor.getSelectedSensorVelocity() / constants.METERS_TO_TICKS, 
            Rotation2d.fromDegrees(angleMotor.getSelectedSensorPosition() / constants.steeringDegreesToTicks));
    }

    /** Stops all motors from running. */
    public void stop() {
        this.driveMotor.set(ControlMode.PercentOutput, 0);
        this.angleMotor.set(ControlMode.PercentOutput, 0);
    }
}
