// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.MPU;
import frc.robot.utils.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
  /** The array of swerve modules on the robot. */
  private final SwerveModule[] modules;

  /** Allows us to calculate the swerve module states from a chassis motion. */
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  private MPU gyro = new MPU();

  public DriveSubsystem() {
    ArrayList<Translation2d> positions = new ArrayList<Translation2d>();
    ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>();

    // Initialize each swerve module with its constants.
    for (SwerveConstants module : Constants.swerveModules) {
      modules.add(new SwerveModule(module));
      positions.add(module.position);
    }

    // Set up the kinematics and odometry.
    this.kinematics = new SwerveDriveKinematics((Translation2d[]) positions.toArray());
    this.modules = (SwerveModule[]) modules.toArray();
    this.odometry = new SwerveDriveOdometry(kinematics, gyro.getHeadingAsRotation2d());

    // Set the default command to the teleoperated command.
    
  }

  /** This method will be called once per scheduler run. */
  @Override
  public void periodic() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for(int x=0; x<modules.length; x++){
      moduleStates[x] = modules[x].getState();
    }
    odometry.update(gyro.getHeadingAsRotation2d(), moduleStates);
  }

  /** Drive the robot in teleoperated mode, relative to the current robot position. */
  public void robotCentricDrive(double forward, double right, double rotation){
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forward, right, rotation);
    SwerveModuleState[] swerveModuleState = kinematics.toSwerveModuleStates(chassisSpeeds);
    for(int x=0; x<modules.length; x++){
      modules[x].drive(swerveModuleState[x].angle.getDegrees(), swerveModuleState[x].speedMetersPerSecond);
    }
  }

  /** Drive the robot in teleoperated mode, relative to the field. */
  public void fieldCentricDrive(double forward, double right, double rotation) {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, right, rotation, gyro.getHeadingAsRotation2d());
    SwerveModuleState[] swerveModuleState = kinematics.toSwerveModuleStates(chassisSpeeds);
    for(int x=0; x<modules.length; x++){
      modules[x].drive(swerveModuleState[x].angle.getDegrees(), swerveModuleState[x].speedMetersPerSecond);
    }
  }

  /** Stop all motors from running. */
  public void stop() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  /** This method will be called once per scheduler run during simulation */
  @Override
  public void simulationPeriodic() {
  }
}
