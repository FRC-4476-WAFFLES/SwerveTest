// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    /** Holds constants like the angle calibration. */
    public final SwerveConstants constants;

    /** The motor controlling the angle of the swerve module. */
    private final CANSparkMax angleMotor;

    /** The motor controlling the speed of the swerve module. */
    private final CANSparkMax driveMotor;

    public SwerveModule(SwerveConstants constants) {
        this.angleMotor = new CANSparkMax(constants.angleMotor, MotorType.kBrushless);
        this.driveMotor = new CANSparkMax(constants.driveMotor, MotorType.kBrushless);
        this.constants = constants;
    }

    /** Drives the swerve module in a direction at a speed.
     * 
     * @param angle The direction to point the swerve module.
     * @param speed The speed of the drive motor. [Range 0..1]
     */
    public void drive(double angle, double speed) {
        // TODO: Point this swerve module in the desired direction, and set
        // the motor speed.
    }

    /** Stops all motors from running. */
    public void stop() {
        this.driveMotor.stopMotor();
        this.angleMotor.stopMotor();
    }
}
