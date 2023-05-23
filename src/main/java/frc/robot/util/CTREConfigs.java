package frc.robot.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.RobotMap.DriveMap;

public final class CTREConfigs {
  public TalonFXConfiguration swerveAngleFXConfig;
  public TalonFXConfiguration swerveDriveFXConfig;
  public CANCoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveAngleFXConfig = new TalonFXConfiguration();
    swerveDriveFXConfig = new TalonFXConfiguration();
    swerveCanCoderConfig = new CANCoderConfiguration();

    /* Swerve Angle Motor Configurations */
    SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
        DriveMap.LIMIT_ROTATOR_CURRENT,
        DriveMap.ROTATOR_CONTINUOS_CURRENT_LIMIT,
        DriveMap.ROTATOR_PEAK_CURRENT_LIMIT,
        DriveMap.ROTATOR_PEAK_CURRENT_DURATION);

    swerveAngleFXConfig.slot0.kP = DriveMap.ROTATOR_KP;
    swerveAngleFXConfig.slot0.kI = DriveMap.ROTATOR_KI;
    swerveAngleFXConfig.slot0.kD = DriveMap.ROTATOR_KD;
    swerveAngleFXConfig.slot0.kF = DriveMap.ROTATOR_KF;
    swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

    /* Swerve Drive Motor Configuration */
    SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        DriveMap.LIMIT_DRIVE_CURRENT,
        DriveMap.DRIVE_CONTINUOS_CURRENT_LIMIT,
        DriveMap.DRIVE_PEAK_CURRENT_LIMIT,
        DriveMap.DRIVE_PEAK_CURRENT_DURATION);

    swerveDriveFXConfig.slot0.kP = DriveMap.DRIVE_KP;
    swerveDriveFXConfig.slot0.kI = DriveMap.DRIVE_KI;
    swerveDriveFXConfig.slot0.kD = DriveMap.DRIVE_KD;
    swerveDriveFXConfig.slot0.kF = DriveMap.DRIVE_KF;
    swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
    swerveDriveFXConfig.openloopRamp = DriveMap.OPEN_LOOP_RAMP;
    swerveDriveFXConfig.closedloopRamp = DriveMap.CLOSED_LOOP_RAMP;

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    swerveCanCoderConfig.sensorDirection = DriveMap.CAN_CODER_INVERT;
    swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }
}
