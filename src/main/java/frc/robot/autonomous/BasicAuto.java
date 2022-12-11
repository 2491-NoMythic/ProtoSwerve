// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BasicAuto extends SequentialCommandGroup {
  /** Creates a new BasicAuto. */
  public BasicAuto(TrajectoryConfig trajectoryConfig, PIDController xController, PIDController yController,
    ProfiledPIDController thetaController, DrivetrainSubsystem drivetrain) {
  
    // Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(5, 5, new Rotation2d(0)),
      List.of(
        new Translation2d(6, 5),
        new Translation2d(6, 4)),
      new Pose2d(7, 4, Rotation2d.fromDegrees(180)),
      trajectoryConfig);

    // Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      drivetrain::getPose,
      Drivetrain.kinematics,
      xController,
      yController,
      thetaController,
      drivetrain::setModuleStates,
      drivetrain);

    addCommands(
      new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> drivetrain.stop()));
  }
}
