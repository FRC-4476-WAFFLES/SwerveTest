// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.MPU;

public class DriveSubsystem extends SubsystemBase {
  SwerveConstants swerveConstants = new SwerveConstants();

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    // Front left
    new Translation2d(SwerveConstants.trackwidth / 2.0, SwerveConstants.wheelbase / 2.0),
    // Front right
    new Translation2d(SwerveConstants.trackwidth / 2.0, -SwerveConstants.wheelbase / 2.0),
    // Back left
    new Translation2d(-SwerveConstants.trackwidth / 2.0, SwerveConstants.wheelbase / 2.0),
    // Back right
    new Translation2d(-SwerveConstants.trackwidth / 2.0, -SwerveConstants.wheelbase / 2.0)
  );

  private MPU gyro = new MPU();

  /** Allows us to calculate the swerve module states from a chassis motion. */
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyro.getHeadingAsRotation2d(), new Pose2d());

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  public DriveSubsystem() {
    frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
      Mk3SwerveModuleHelper.GearRatio.STANDARD,
      swerveConstants.driveMotor[0],
      swerveConstants.angleMotor[0],
      swerveConstants.angleMotor[0],
      swerveConstants.offset[0]
      );

    // We will do the same for the other modules
    frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
      Mk3SwerveModuleHelper.GearRatio.STANDARD,
      swerveConstants.driveMotor[1],
      swerveConstants.angleMotor[1],
      swerveConstants.angleMotor[1],
      swerveConstants.offset[1]
    );

    backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
      Mk3SwerveModuleHelper.GearRatio.STANDARD,
      swerveConstants.driveMotor[2],
      swerveConstants.angleMotor[2],
      swerveConstants.angleMotor[2],
      swerveConstants.offset[2]
    );

    backRightModule = Mk3SwerveModuleHelper.createFalcon500(
      Mk3SwerveModuleHelper.GearRatio.STANDARD,
      swerveConstants.driveMotor[3],
      swerveConstants.angleMotor[3],
      swerveConstants.angleMotor[3],
      swerveConstants.offset[3]
    );
  }

  /** This method will be called once per scheduler run. */
  @Override
  public void periodic() {
    gyro.update();
  }

  public void robotDrive(double forward, double right, double rotation, boolean fieldCentric){
    ChassisSpeeds chassisSpeeds;

    if (Math.abs(forward) < .05){
      forward = 0;
    }
    if(Math.abs(right) < .05){
      right = 0;
    }
    if(Math.abs(rotation) < .05){
      rotation = 0;
    }

    forward *= -1;
    right *= -1;
    rotation *= -1;

    if (fieldCentric){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, right, rotation, gyro.getHeadingAsRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(forward, right, rotation);
    }
    
    SwerveModuleState[] swerveModuleState = kinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(swerveModuleState);
    SmartDashboard.putNumber("Gyro", gyro.getHeading());
    SmartDashboard.putNumber("FL Commanded Speed", swerveModuleState[0].speedMetersPerSecond);
    SmartDashboard.putNumber("FL Actual Speed", frontLeftModule.getDriveVelocity());
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates){
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, SwerveConstants.maxVelocity);

    frontLeftModule.set(swerveModuleStates[0].speedMetersPerSecond / SwerveConstants.maxVelocity * SwerveConstants.maxVoltage, swerveModuleStates[0].angle.getRadians());
    frontRightModule.set(swerveModuleStates[1].speedMetersPerSecond / SwerveConstants.maxVelocity * SwerveConstants.maxVoltage, swerveModuleStates[1].angle.getRadians());
    backLeftModule.set(swerveModuleStates[2].speedMetersPerSecond / SwerveConstants.maxVelocity * SwerveConstants.maxVoltage, swerveModuleStates[2].angle.getRadians());
    backRightModule.set(swerveModuleStates[3].speedMetersPerSecond / SwerveConstants.maxVelocity * SwerveConstants.maxVoltage, swerveModuleStates[3].angle.getRadians());
  }

  public Pose2d getOdometryLocation(){
    return odometry.getPoseMeters();
  }

  /** Stop all motors from running. */
  public void stop() {
    frontLeftModule.set(0, frontLeftModule.getSteerAngle());
    frontRightModule.set(0, frontRightModule.getSteerAngle());
    backLeftModule.set(0, backRightModule.getSteerAngle());
    backRightModule.set(0, backRightModule.getSteerAngle());
  }

  /** This method will be called once per scheduler run during simulation */
  @Override
  public void simulationPeriodic() {
  }
}
