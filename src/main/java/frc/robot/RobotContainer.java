// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.PS4.*;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
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
import frc.robot.autonomous.*;

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
  private final SendableChooser<Command> autoChooser;
  private final PS4Controller controller = new PS4Controller(0);
  
  public PIDController xController;
  public PIDController yController;
  public ProfiledPIDController thetaController;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // SmartDashboard.putData("xController", xController);
    // SmartDashboard.putData("yController", yController);
    // SmartDashboard.putData("thetaController", thetaController);

    autoChooser = new SendableChooser<>();

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    defaultDriveCommand = new Drive(
      drivetrain,
      () -> controller.getL1Button(),
      () -> controller.getR1Button(),
      () -> modifyAxis(-controller.getRawAxis(X_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-controller.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-controller.getRawAxis(Z_AXIS), DEADBAND_NORMAL),
      () -> getJoystickDegrees(Z_AXIS, Z_ROTATE),
      () -> getJoystickMagnitude(Z_AXIS, Z_ROTATE));

    drivetrain.setDefaultCommand(defaultDriveCommand);
    SmartDashboard.putNumber("kPxy", 1.5);
    SmartDashboard.putNumber("kIxy", 0);
    SmartDashboard.putNumber("kDxy", 0);
    SmartDashboard.putNumber("kPtheta", 3);
    SmartDashboard.putNumber("kItheta", 0);
    SmartDashboard.putNumber("kDtheta", 0);
    SmartDashboard.setPersistent("kPxy");
    SmartDashboard.setPersistent("kIxy");
    SmartDashboard.setPersistent("kDxy");
    SmartDashboard.setPersistent("kPtheta");
    SmartDashboard.setPersistent("kItheta");
    SmartDashboard.setPersistent("kDtheta");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the button bindings
    configureButtonBindings();
  }
  /**Takes both axis of a joystick, returns an angle from -180 to 180 degrees, or {@link Constants.PS4.NO_INPUT} (double = 404.0) if the joystick is at rest position*/
  private double getJoystickDegrees(int horizontalAxis, int verticalAxis) {
    double xAxis = deadband(-controller.getRawAxis(horizontalAxis), DEADBAND_LARGE);
    double yAxis = deadband(-controller.getRawAxis(verticalAxis), DEADBAND_LARGE);
    if (xAxis + yAxis != 0) {
      return Math.toDegrees(Math.atan2(xAxis, yAxis));
    }
    return NO_INPUT;
  }
  /**Takes both axis of a joystick, returns a double from 0-1 */
  private double getJoystickMagnitude(int horizontalAxis, int verticalAxis) {
    double xAxis = deadband(-controller.getRawAxis(horizontalAxis), DEADBAND_NORMAL);
    double yAxis = deadband(-controller.getRawAxis(verticalAxis), DEADBAND_NORMAL);
    return Math.min(1.0, (Math.sqrt(Math.pow(xAxis, 2) + Math.pow(yAxis, 2)))); // make sure the number is not greater than 1
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
    return autoChooser.getSelected();
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
    autoChooser.setDefaultOption("Basic Auto", new BasicAuto(xController, yController, thetaController, drivetrain));
  }
  public void teleopInit() {
    drivetrain.pointWheelsForward();
  }
  public void autoInit() {
    xController = new PIDController(
      SmartDashboard.getNumber("kPxy", 1.5),
      SmartDashboard.getNumber("kIxy", 0),
      SmartDashboard.getNumber("kDxy", 0));
    yController = new PIDController(
      SmartDashboard.getNumber("kPxy", 1.5),
      SmartDashboard.getNumber("kIxy", 0),
      SmartDashboard.getNumber("kDxy", 0));
    thetaController = new ProfiledPIDController(
      SmartDashboard.getNumber("kPtheta", 3),
      SmartDashboard.getNumber("kItheta", 0),
      SmartDashboard.getNumber("kDtheta", 0),
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }
}