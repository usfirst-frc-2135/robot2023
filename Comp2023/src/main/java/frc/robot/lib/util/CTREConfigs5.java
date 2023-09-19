package frc.robot.lib.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants.SWConsts;

public final class CTREConfigs5
{
  // Grouped by subsystem

  // TODO: remove when swerve testing is complete on v6 -- should gripper have a config?

  // Swerve modules

  // public static TalonFXConfiguration swerveDriveFXConfig( )
  // {
  //   TalonFXConfiguration config = new TalonFXConfiguration( );
  //   SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(SWConsts.driveEnableCurrentLimit,
  //       SWConsts.driveContinuousCurrentLimit, SWConsts.drivePeakCurrentLimit, SWConsts.drivePeakCurrentDuration);

  //   config.slot0.kP = SWConsts.driveKP;
  //   config.slot0.kI = SWConsts.driveKI;
  //   config.slot0.kD = SWConsts.driveKD;
  //   config.slot0.kF = SWConsts.driveKF;
  //   config.supplyCurrLimit = driveSupplyLimit;
  //   config.initializationStrategy = SensorInitializationStrategy.BootToZero;
  //   config.openloopRamp = SWConsts.driveOpenLoopRamp;
  //   config.closedloopRamp = SWConsts.driveClosedLoopRamp;
  //   return config;
  // }

  // public static TalonFXConfiguration swerveAngleFXConfig( )
  // {
  //   TalonFXConfiguration angleConfig = new TalonFXConfiguration( );
  //   SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(SWConsts.angleEnableCurrentLimit,
  //       SWConsts.angleContinuousCurrentLimit, SWConsts.anglePeakCurrentLimit, SWConsts.anglePeakCurrentDuration);

  //   angleConfig.slot0.kP = SWConsts.angleKP;
  //   angleConfig.slot0.kI = SWConsts.angleKI;
  //   angleConfig.slot0.kD = SWConsts.angleKD;
  //   angleConfig.slot0.kF = SWConsts.angleKF;
  //   angleConfig.supplyCurrLimit = angleSupplyLimit;
  //   angleConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
  //   return angleConfig;
  // }

  // public static CANCoderConfiguration swerveCancoderConfig( )
  // {
  //   CANCoderConfiguration config = new CANCoderConfiguration( );
  //   config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
  //   config.sensorDirection = SWConsts.canCoderInvert;
  //   config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
  //   config.sensorTimeBase = SensorTimeBase.PerSecond;
  //   return config;
  // }

}
