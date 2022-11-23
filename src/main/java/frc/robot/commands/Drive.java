package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.PS4;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Drive extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final BooleanSupplier robotCentricMode;
    private final BooleanSupplier fieldCentricRotateMode;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private final DoubleSupplier turnSupplier;
    private final PIDController turnController;
    
    public Drive(DrivetrainSubsystem drivetrainSubsystem,
    BooleanSupplier robotCentricMode,
    BooleanSupplier fieldCentricRotateMode,
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationSupplier,
    DoubleSupplier turnSupplier) {
        this.drivetrain = drivetrainSubsystem;
        this.robotCentricMode = robotCentricMode;
        this.fieldCentricRotateMode = fieldCentricRotateMode;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.turnSupplier = turnSupplier;
        turnController = new PIDController(Drivetrain.K_TURN_P, Drivetrain.K_TURN_I, Drivetrain.K_TURN_D);
        turnController.enableContinuousInput(-180.0, 180.0);
        turnController.setTolerance(Drivetrain.K_TURN_TOLORANCE_DEGREES, Drivetrain.K_TURN_TOLORANCE_DEG_PER_SEC);
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if (robotCentricMode.getAsBoolean()) {
            drivetrain.drive(new ChassisSpeeds(
                translationXSupplier.getAsDouble() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                translationYSupplier.getAsDouble() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                rotationSupplier.getAsDouble() * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            ));
        } else if (fieldCentricRotateMode.getAsBoolean() && turnSupplier.getAsDouble() != PS4.NO_INPUT) {
            drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translationXSupplier.getAsDouble() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                    translationYSupplier.getAsDouble() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                    MathUtil.clamp(turnController.calculate(drivetrain.getGyroscopeRotation().getDegrees(), turnSupplier.getAsDouble()), -1, 1) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    drivetrain.getGyroscopeRotation()
                )
            );
        } else {
            drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translationXSupplier.getAsDouble() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                    translationYSupplier.getAsDouble() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                    rotationSupplier.getAsDouble() * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    drivetrain.getGyroscopeRotation()
                )
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}