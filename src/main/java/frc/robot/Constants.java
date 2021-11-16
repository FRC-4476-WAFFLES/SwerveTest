// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class SwerveConstants {
        public final int driveMotor[] = new int[]{4, 6, 2, 8};
        public final int angleMotor[] = new int[]{3, 5, 1, 7};
        public final double offset[] = new double[]{0, 0, 0, 0};
        
        public static final double trackwidth = 0.5588;
        public static final double wheelbase = 0.5588;
        public static final double maxVoltage = 12.0;
        public static final double maxVelocity = 6380.0 / 60.0 * SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() * SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
        public static final double maxAngularVelocity = maxVelocity / Math.hypot(trackwidth / 2.0, wheelbase / 2.0);
    }

    public static final class MPUConstants{
        public final double calibration = 0;
        public final int address = 0x68;
    }
}
