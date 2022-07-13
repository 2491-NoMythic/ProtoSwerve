package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drivetrain;
import static frc.robot.Constants.PS4.*;
import static frc.robot.Constants.Drivetrain.*;

public class Drive extends CommandBase {
    private Drivetrain drivetrain;

    double speedManager;
    boolean fieldRelative;
    PS4Controller controller = new PS4Controller(CONTROLLER_ID);

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter forwardSpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter sidewaysSpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);

    public Drive(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("SpeedManager", 1);
        SmartDashboard.putBoolean("IsFieldRelative", false);
    }

    @Override
    public void execute() {
        speedManager = SmartDashboard.getNumber("SpeedManager", 1);
        fieldRelative = SmartDashboard.getBoolean("IsFieldRelative", true);
        // Get the x speed. We are inverting this because PS4 controllers return
        // negative values when we push forward.
        final var forwardSpeed =
            -forwardSpeedLimiter.calculate(MathUtil.applyDeadband(controller.getRawAxis(Y_AXES), 0.02))
                * K_MAX_SPEED;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. PS4 controllers
        // return positive values when you pull to the right by default.
        final var sidewaysSpeed =
            -sidewaysSpeedLimiter.calculate(MathUtil.applyDeadband(controller.getRawAxis(X_AXES), 0.02))
                * K_MAX_SPEED;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). PS4 controllers return positive values when you pull to
        // the right by default.
        final var rotation =
            -rotationLimiter.calculate(MathUtil.applyDeadband(controller.getRawAxis(Z_AXES), 0.02))
                * K_MAX_ANGULAR_SPEED * speedManager;

        drivetrain.drive(forwardSpeed, sidewaysSpeed, rotation, fieldRelative);
    }
}
