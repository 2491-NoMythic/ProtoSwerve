package frc.robot;

import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.CANcoderConfigurator;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import static frc.robot.Constants.DriveConstants.*;

public class CTREConfigs {
    public TalonFXConfiguration driveMotorConfig;
    public TalonFXConfiguration steerMotorConfig;
    public CANcoderConfiguration steerEncoderConfig;
    public Pigeon2Configuration pigeon2Config;

    public CTREConfigs() {
        driveMotorConfig = new TalonFXConfiguration();
        steerMotorConfig = new TalonFXConfiguration();
        steerEncoderConfig = new CANcoderConfiguration();
        pigeon2Config = new Pigeon2Configuration();

        // Steer motor.
        steerMotorConfig.Feedback.RotorToSensorRatio = 1/DRIVETRAIN_STEER_REDUCTION;
        steerMotorConfig.MotorOutput.Inverted = DRIVETRAIN_STEER_INVERTED;
        steerMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.05;
        steerMotorConfig.Slot0.kP = K_STEER_P;
        steerMotorConfig.Slot0.kI = K_STEER_I;
        steerMotorConfig.Slot0.kD = K_STEER_D;
        steerMotorConfig.Slot0.kS = K_STEER_FF_S;
        steerMotorConfig.Slot0.kV = K_STEER_FF_V;
        steerMotorConfig.Voltage.PeakForwardVoltage = 12;
        steerMotorConfig.Voltage.PeakReverseVoltage = -12;
        steerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
        // Drive motor.
        driveMotorConfig.Feedback.SensorToMechanismRatio = 1/DRIVETRAIN_DRIVE_REDUCTION;
        driveMotorConfig.MotorOutput.Inverted = DRIVETRAIN_DRIVE_INVERTED;
        driveMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.0;
        driveMotorConfig.Slot0.kP = K_DRIVE_P;
        driveMotorConfig.Slot0.kI = K_DRIVE_I;
        driveMotorConfig.Slot0.kD = K_DRIVE_D;
        driveMotorConfig.Slot0.kS = K_DRIVE_FF_S;
        driveMotorConfig.Slot0.kV = K_DRIVE_FF_V;
        driveMotorConfig.Voltage.PeakForwardVoltage = 12;
        driveMotorConfig.Voltage.PeakReverseVoltage = -12;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //  Steer encoder.
        steerEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        // Pigeon 2.
        pigeon2Config.MountPose.MountPosePitch = 0;
        pigeon2Config.MountPose.MountPoseRoll = 0;
        pigeon2Config.MountPose.MountPoseYaw = 0;
    }
}
