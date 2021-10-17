// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    /** Holds constants like the angle calibration. */
    public final SwerveConstants constants;

    /** The motor controlling the angle of the swerve module. */
    private final TalonFX angleMotor;

    /** The motor controlling the speed of the swerve module. */
    private final TalonFX driveMotor;

    public SwerveModule(SwerveConstants constants) {
        this.angleMotor = new TalonFX(constants.angleMotor);
        this.driveMotor = new TalonFX(constants.driveMotor);
        this.constants = constants;
    }

    /** Drives the swerve module in a direction at a speed.
     * 
     * @param angle The direction to point the swerve module.
     * @param speed The speed of the drive motor. [Range 0..1]
     */
    public void drive(double angle, double speed) {
        driveMotor.set(ControlMode.PercentOutput, speed);
        double currentAngle = angleMotor.getSelectedSensorPosition() / 72.81;
        currentAngle = currentAngle % 360;
        
        angle *= 1;
        double turnSpeed = 0;

        if (angle - currentAngle > 180){
            angle -= 360;
        }
        
        if (currentAngle + 2 < angle){
            turnSpeed = (angle - currentAngle) / 120;
        } else if(currentAngle - 2 > angle){
            turnSpeed = (angle - currentAngle) / 120;
        } else {
            turnSpeed = 0;
        }

        if (turnSpeed > 0.5){
            turnSpeed = 0.5;
        } else if (turnSpeed < -0.5){
            turnSpeed = -0.5;
        }
        
        angleMotor.set(ControlMode.PercentOutput, turnSpeed);
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
