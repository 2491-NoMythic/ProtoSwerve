// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.PS4.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
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
    // SmartDashboard.putNumber("kPxy", 1.5);
    // SmartDashboard.putNumber("kIxy", 0);
    // SmartDashboard.putNumber("kDxy", 0);
    // SmartDashboard.putNumber("kPtheta", 3);
    // SmartDashboard.putNumber("kItheta", 0);
    // SmartDashboard.putNumber("kDtheta", 0);
    // SmartDashboard.setPersistent("kPxy");
    // SmartDashboard.setPersistent("kIxy");
    // SmartDashboard.setPersistent("kDxy");
    // SmartDashboard.setPersistent("kPtheta");
    // SmartDashboard.setPersistent("kItheta");
    // SmartDashboard.setPersistent("kDtheta");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("DriveTurn180", autoBuilder.fullAuto(pathGroup));
    // SmartDashboard.putData("Basic Auto", new BasicAuto(xController, yController, thetaController, drivetrain));
    // SmartDashboard.putData("Forward Turn Auto", new ForwardsTurn180(xController, yController, thetaController, drivetrain));
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
    new Trigger(controller::getPSButton).onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));
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
    // autoChooser.setDefaultOption([Auto Goes Here]);
  }
  public void teleopInit() {
    drivetrain.pointWheelsForward();
  }

  
  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands.
  HashMap<String, Command> eventMap = new HashMap<>();
  
  // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
  SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
  drivetrain::getPose, // Pose2d supplier
  drivetrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
  drivetrain.kinematics, // SwerveDriveKinematics
  new PIDConstants(
    DriveConstants.K_XY_P,
      DriveConstants.K_XY_I,
      DriveConstants.K_XY_D), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(
        DriveConstants.K_THETA_P,
        DriveConstants.K_THETA_I,
        DriveConstants.K_THETA_D), // PID constants to correct for rotation error (used to create the rotation controller)
        drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
        );
        
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Forward180", new PathConstraints(2, 3));
    Command fullAuto = autoBuilder.fullAuto(pathGroup);
    
}