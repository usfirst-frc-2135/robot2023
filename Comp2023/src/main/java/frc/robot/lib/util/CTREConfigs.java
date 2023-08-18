package frc.robot.lib.util;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants;
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.EXConsts;
import frc.robot.Constants.WRConsts;

public final class CTREConfigs
{
  public static TalonFXConfiguration swerveDriveFXConfig( )
  {
    TalonFXConfiguration config = new TalonFXConfiguration( );
    SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        Constants.SwerveConstants.driveEnableCurrentLimit, Constants.SwerveConstants.driveContinuousCurrentLimit,
        Constants.SwerveConstants.drivePeakCurrentLimit, Constants.SwerveConstants.drivePeakCurrentDuration);

    config.slot0.kP = Constants.SwerveConstants.driveKP;
    config.slot0.kI = Constants.SwerveConstants.driveKI;
    config.slot0.kD = Constants.SwerveConstants.driveKD;
    config.slot0.kF = Constants.SwerveConstants.driveKF;
    config.supplyCurrLimit = driveSupplyLimit;
    config.initializationStrategy = SensorInitializationStrategy.BootToZero;
    config.openloopRamp = Constants.SwerveConstants.openLoopRamp;
    config.closedloopRamp = Constants.SwerveConstants.closedLoopRamp;
    return config;
  }

  public static TalonFXConfiguration swerveAngleFXConfig( )
  {
    TalonFXConfiguration angleConfig = new TalonFXConfiguration( );
    SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
        Constants.SwerveConstants.angleEnableCurrentLimit, Constants.SwerveConstants.angleContinuousCurrentLimit,
        Constants.SwerveConstants.anglePeakCurrentLimit, Constants.SwerveConstants.anglePeakCurrentDuration);

    angleConfig.slot0.kP = Constants.SwerveConstants.angleKP;
    angleConfig.slot0.kI = Constants.SwerveConstants.angleKI;
    angleConfig.slot0.kD = Constants.SwerveConstants.angleKD;
    angleConfig.slot0.kF = Constants.SwerveConstants.angleKF;
    angleConfig.supplyCurrLimit = angleSupplyLimit;
    angleConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    return angleConfig;
  }

  public static TalonFXConfiguration wristAngleFXConfig( )
  {
    TalonFXConfiguration angleConfig = new TalonFXConfiguration( );
    SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(true,
        Constants.WRConsts.kSupplyCurrentLimit, Constants.WRConsts.kSupplyTriggerCurrent, Constants.WRConsts.kSupplyTriggerTime);
    StatorCurrentLimitConfiguration angleStatorCurrentLimits = new StatorCurrentLimitConfiguration(true,
        WRConsts.kStatorCurrentLimit, WRConsts.kStatorTriggerCurrent, WRConsts.kStatorTriggerTime);

    angleConfig.slot0.kP = Constants.WRConsts.kPidKp;
    angleConfig.slot0.kI = Constants.WRConsts.kPidKi;
    angleConfig.slot0.kD = Constants.WRConsts.kPidKd;
    angleConfig.slot0.kF = Constants.WRConsts.kPidKf;

    angleConfig.supplyCurrLimit = angleSupplyLimit;
    angleConfig.statorCurrLimit = angleStatorCurrentLimits;

    angleConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    angleConfig.voltageCompSaturation = 12.0;

    angleConfig.motionCruiseVelocity = WRConsts.kMMVelocity;
    angleConfig.motionAcceleration = WRConsts.kMMAcceleration;
    angleConfig.motionCurveStrength = WRConsts.kMMSCurveStrength;

    angleConfig.neutralDeadband = WRConsts.kNeutralDeadband;

    return angleConfig;
  }

  public static TalonFXConfiguration elbowAngleFXConfig( )
  {
    TalonFXConfiguration angleConfig = new TalonFXConfiguration( );
    SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(true,
        Constants.ELConsts.kSupplyCurrentLimit, Constants.ELConsts.kSupplyTriggerCurrent, Constants.ELConsts.kSupplyTriggerTime);
    StatorCurrentLimitConfiguration angleStatorCurrentLimits = new StatorCurrentLimitConfiguration(true,
        ELConsts.kStatorCurrentLimit, ELConsts.kStatorTriggerCurrent, ELConsts.kStatorTriggerTime);

    angleConfig.slot0.kP = Constants.ELConsts.kPidKp;
    angleConfig.slot0.kI = Constants.ELConsts.kPidKi;
    angleConfig.slot0.kD = Constants.ELConsts.kPidKd;
    angleConfig.slot0.kF = Constants.ELConsts.kPidKf;

    angleConfig.supplyCurrLimit = angleSupplyLimit;
    angleConfig.statorCurrLimit = angleStatorCurrentLimits;

    angleConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    angleConfig.voltageCompSaturation = 12.0;

    angleConfig.motionCruiseVelocity = ELConsts.kMMVelocity;
    angleConfig.motionAcceleration = ELConsts.kMMAcceleration;
    angleConfig.motionCurveStrength = ELConsts.kMMSCurveStrength;

    angleConfig.neutralDeadband = ELConsts.kNeutralDeadband;

    return angleConfig;
  }

  public static TalonFXConfiguration extensionLengthFXConfig( )
  {
    TalonFXConfiguration lengthConfig = new TalonFXConfiguration( );
    SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(true, EXConsts.kSupplyCurrentLimit,
        EXConsts.kSupplyTriggerCurrent, EXConsts.kSupplyTriggerTime);
    StatorCurrentLimitConfiguration angleStatorCurrentLimits = new StatorCurrentLimitConfiguration(true,
        EXConsts.kStatorCurrentLimit, EXConsts.kStatorTriggerCurrent, EXConsts.kStatorTriggerTime);

    lengthConfig.slot0.kP = Constants.EXConsts.kPidKp;
    lengthConfig.slot0.kI = Constants.EXConsts.kPidKi;
    lengthConfig.slot0.kD = Constants.EXConsts.kPidKd;
    lengthConfig.slot0.kF = Constants.EXConsts.kPidKf;

    lengthConfig.supplyCurrLimit = angleSupplyLimit;
    lengthConfig.statorCurrLimit = angleStatorCurrentLimits;

    lengthConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    lengthConfig.voltageCompSaturation = 12.0;

    lengthConfig.motionCruiseVelocity = EXConsts.kMMVelocity;
    lengthConfig.motionAcceleration = EXConsts.kMMAcceleration;
    lengthConfig.motionCurveStrength = EXConsts.kMMSCurveStrength;

    lengthConfig.neutralDeadband = EXConsts.kNeutralDeadband;

    return lengthConfig;
  }

  public static CANCoderConfiguration swerveCancoderConfig( )
  {
    CANCoderConfiguration config = new CANCoderConfiguration( );
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.sensorDirection = Constants.SwerveConstants.canCoderInvert;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    return config;
  }

  public static CANCoderConfiguration elbowCancoderConfig( )
  {
    CANCoderConfiguration config = new CANCoderConfiguration( );
    config.magnetOffsetDegrees = (Constants.isComp) ? ELConsts.kCompOffset : ELConsts.kBetaOffset;
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.sensorDirection = Constants.ELConsts.kInvertCANCoder;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    return config;
  }

  public static CANCoderConfiguration wristCancoderConfig( )
  {
    CANCoderConfiguration config = new CANCoderConfiguration( );
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.magnetOffsetDegrees = (Constants.isComp) ? WRConsts.kCompOffset : WRConsts.kBetaOffset;
    config.sensorDirection = Constants.WRConsts.kInvertCANCoder;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    return config;
  }

}
