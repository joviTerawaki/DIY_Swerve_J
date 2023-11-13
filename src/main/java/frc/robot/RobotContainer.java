// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.S_DriveCommand;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class RobotContainer {
  private final XboxController xbox = new XboxController(ControllerConstants.kDriverControllerPort);

  //auto choices 


  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(
      new S_DriveCommand(swerveSubsystem, () -> -xbox.getLeftX(), () -> xbox.getLeftY(), () -> xbox.getRightX(), false)
    );

    //REGISTER NAMED COMMANDS FOR AUTOS SO THEY WORK :) 
    //use class NamedCommands and static method registerCommand 

    configureBindings();
  }

  private void configureBindings() {
    //new JoystickButton(xbox, 1).onTrue(new InstantCommand(() -> swerveSubsystem.lock(), swerveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return null; 
    /* 
    //CONFIG FOR TRAJECTORY 
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(SwerveConstants.MAX_SPEED, 1).setKinematics(SwerveConstants.DRIVE_KINEMATICS);

    //GENERATING THE TRAJECTORY 
    // (initial position (Pose2d), interior waypoints (Translation2d), ending position (Pose2d), trajectory configuration)
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),  //initial position 
      List.of(
        new Translation2d(1, 0)  //waypoint 1
      ), 
      new Pose2d(1, 1, Rotation2d.fromDegrees(0)),  //ending position 
      trajectoryConfig
      );

    // PID FOR TRAJECTORY 
    PIDController xController = new PIDController(0.1, 0, 0);
    PIDController yController = new PIDController(0.1, 0, 0); 

    ProfiledPIDController angleController = new ProfiledPIDController(0.1, 0, 0, new Constraints(SwerveConstants.MAX_ROTATION, 1));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    //FOLLOWING THE TRAJECTORY USING A SWERVE CONTROLLER COMMAND 
    //traj obj, get odometer, kinematics, pidControllers, set module states, swerve subs 
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory, 
      swerveSubsystem::getPose, 
      SwerveConstants.DRIVE_KINEMATICS, 
      xController, yController, angleController, 
      swerveSubsystem::setModuleStates, 
      swerveSubsystem
      );

    //RETURN COMMAND SO IT'S ALL CALLED 
    return new SequentialCommandGroup(
      new InstantCommand( () -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), 
      swerveControllerCommand, 
      new InstantCommand( () -> swerveSubsystem.stopModules())
    );
    */
  }
}
