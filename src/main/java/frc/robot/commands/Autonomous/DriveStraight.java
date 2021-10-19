// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraight extends SwerveControllerCommand {
  /** Creates a new DriveStraight. */
  public DriveStraight(DriveSubsystem driveSubsystem, Trajectory trajectory) {
    super(trajectory, 
    driveSubsystem::getOdometryLocation, 
    driveSubsystem.kinematics, 
    new PIDController(0.01, 0, 0), 
    new PIDController(0.01, 0, 0), 
    new ProfiledPIDController(0.01, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.1)), 
    driveSubsystem::setModuleStates, 
    driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

}