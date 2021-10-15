package frc.robot.utils;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.Constants.MPUConstants;

public class MPU {
    private final MPUConstants constants = new MPUConstants();
    private final I2C MPU = new I2C(I2C.Port.kOnboard, constants.address);

    public MPU(){
        MPU.write(0x6B, 0);
    }
    public double getHeading(){
        byte[] byteArray = new byte[8];
        boolean wasAborted = MPU.read(0x47, 8, byteArray);
        return ByteBuffer.wrap(byteArray).getDouble();
    }
    public Rotation2d getHeadingAsRotation2d(){
        byte[] byteArray = new byte[8];
        boolean wasAborted = MPU.read(0x47, 8, byteArray);
        return Rotation2d.fromDegrees(ByteBuffer.wrap(byteArray).getDouble());
    }
}
