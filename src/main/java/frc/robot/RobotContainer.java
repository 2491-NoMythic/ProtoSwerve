// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.PS4.*;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Drivetrain;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final Drive defaultDriveCommand;
  private final PS4Controller controller = new PS4Controller(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    defaultDriveCommand = new Drive(
      drivetrain,
      () -> controller.getL1Button(),
      () -> controller.getR1Button(),
      () -> modifyAxis(-controller.getRawAxis(X_AXES), DEADBAND_NORMAL),
      () -> modifyAxis(-controller.getRawAxis(Y_AXES), DEADBAND_NORMAL),
      () -> modifyAxis(-controller.getRawAxis(Z_AXES), DEADBAND_NORMAL),
      () -> getJoystickDegrees());

    drivetrain.setDefaultCommand(defaultDriveCommand);

    // Configure the button bindings
    configureButtonBindings();
  }
  private double getJoystickDegrees() {
    double zAxis = modifyAxis(-controller.getRawAxis(Z_AXES), DEADBAND_LARGE);
    double zRotate = modifyAxis(-controller.getRawAxis(Z_ROTATE), DEADBAND_LARGE);
    if (zAxis + zRotate != 0) {
      return Math.toDegrees(Math.atan2(zAxis, zRotate));
    }
    return NO_INPUT;
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    // new Button(() -> controller.getRawButton(13)).whenPressed(drivetrain::zeroGyroscope);
    new Button(controller::getPSButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(drivetrain::zeroGyroscope);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Drivetrain.kinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(5, 5, new Rotation2d(0)),
            List.of(
                    new Translation2d(6, 5),
                    new Translation2d(6, 4)),
            new Pose2d(7, 4, Rotation2d.fromDegrees(180)),
            trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            drivetrain::getPose,
            Drivetrain.kinematics,
            xController,
            yController,
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> drivetrain.stop()));
}

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value, double deadband) {
    // Deadband
    value = deadband(value, deadband);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  public void robotInit() {
    drivetrain.zeroGyroscope();
  }
  public void teleopInit() {
    drivetrain.pointWheelsForward();
  }
}