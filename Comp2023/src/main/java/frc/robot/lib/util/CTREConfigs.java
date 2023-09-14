package frc.robot.lib.util;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import frc.robot.Constants;
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.EXConsts;
import frc.robot.Constants.WRConsts;
import frc.robot.lib.math.Conversions;

public final class CTREConfigs
{
  // Grouped by subsystem

  // Swerve modules

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

  public static CANCoderConfiguration swerveCancoderConfig( )
  {
    CANCoderConfiguration config = new CANCoderConfiguration( );
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.sensorDirection = Constants.SwerveConstants.canCoderInvert;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    return config;
  }

  // Elbow
  /*
   * public static TalonFXConfiguration elbowMotorFXConfig( )
   * {
   * TalonFXConfiguration elbowConfig = new TalonFXConfiguration( );
   * 
   * elbowConfig.Slot0.kP = Constants.ELConsts.kPidKp;
   * elbowConfig.Slot0.kI = Constants.ELConsts.kPidKi;
   * elbowConfig.Slot0.kD = Constants.ELConsts.kPidKd;
   * elbowConfig.Slot0.kV = Constants.ELConsts.kPidKf;
   * 
   * elbowConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
   * elbowConfig.CurrentLimits.SupplyCurrentLimit = Constants.ELConsts.kSupplyCurrentLimit;
   * elbowConfig.CurrentLimits.SupplyCurrentThreshold = Constants.ELConsts.kSupplyTriggerCurrent;
   * elbowConfig.CurrentLimits.SupplyTimeThreshold = Constants.ELConsts.kSupplyTriggerTime;
   * 
   * elbowConfig.CurrentLimits.StatorCurrentLimitEnable = true;
   * elbowConfig.CurrentLimits.StatorCurrentLimit = ELConsts.kStatorCurrentLimit;
   * 
   * elbowConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
   * elbowConfig.voltageCompSaturation = 12.0;
   * 
   * elbowConfig.motionCruiseVelocity = ELConsts.kMMVelocity;
   * elbowConfig.motionAcceleration = ELConsts.kMMAcceleration;
   * elbowConfig.motionCurveStrength = ELConsts.kMMSCurveStrength;
   * 
   * elbowConfig.reverseSoftLimitThreshold = Conversions.degreesToFalcon(ELConsts.kAngleMin,
   * ELConsts.kGearRatio);
   * elbowConfig.reverseSoftLimitEnable = true;
   * elbowConfig.forwardSoftLimitThreshold = Conversions.degreesToFalcon(ELConsts.kAngleMax,
   * ELConsts.kGearRatio);
   * elbowConfig.forwardSoftLimitEnable = true;
   * 
   * elbowConfig.slot0.allowableClosedloopError = ELConsts.kAllowedError;
   * 
   * elbowConfig.neutralDeadband = ELConsts.kNeutralDeadband;
   * 
   * return elbowConfig;
   * }
   */
  // public static CANCoderConfiguration elbowCancoderConfig( )
  // {
  //   CANCoderConfiguration config = new CANCoderConfiguration( );
  //   config.magnetOffsetDegrees = (Constants.isComp) ? ELConsts.kCompOffset : ELConsts.kBetaOffset;
  //   config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
  //   config.sensorDirection = Constants.ELConsts.kInvertCANCoder;
  //   config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
  //   config.sensorTimeBase = SensorTimeBase.PerSecond;
  //   return config;
  // }

  // Extension

  public static TalonFXConfiguration extensionLengthFXConfig( )
  {
    TalonFXConfiguration extensionConfig = new TalonFXConfiguration( );

    extensionConfig.slot0.kP = Constants.EXConsts.kPidKp;
    extensionConfig.slot0.kI = Constants.EXConsts.kPidKi;
    extensionConfig.slot0.kD = Constants.EXConsts.kPidKd;
    extensionConfig.slot0.kF = Constants.EXConsts.kV;

    extensionConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, EXConsts.kSupplyCurrentLimit,
        EXConsts.kSupplyTriggerCurrent, EXConsts.kSupplyTriggerTime);
    extensionConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, EXConsts.kStatorCurrentLimit,
        EXConsts.kStatorTriggerCurrent, EXConsts.kStatorTriggerTime);

    extensionConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    extensionConfig.voltageCompSaturation = 12.0;

    extensionConfig.motionCruiseVelocity = EXConsts.kMMVelocity;
    extensionConfig.motionAcceleration = EXConsts.kMMAcceleration;
    extensionConfig.motionCurveStrength = EXConsts.kMMJerk;

    extensionConfig.neutralDeadband = EXConsts.kNeutralDeadband;

    extensionConfig.slot0.allowableClosedloopError = EXConsts.kAllowedError;

    return extensionConfig;
  }

  // Wrist

  public static TalonFXConfiguration wristAngleFXConfig( )
  {
    TalonFXConfiguration wristConfig = new TalonFXConfiguration( );

    wristConfig.slot0.kP = Constants.WRConsts.kPidKp;
    wristConfig.slot0.kI = Constants.WRConsts.kPidKi;
    wristConfig.slot0.kD = Constants.WRConsts.kPidKd;
    wristConfig.slot0.kF = Constants.WRConsts.kV;

    wristConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, Constants.WRConsts.kSupplyCurrentLimit,
        Constants.WRConsts.kSupplyTriggerCurrent, Constants.WRConsts.kSupplyTriggerTime);
    wristConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, WRConsts.kStatorCurrentLimit,
        WRConsts.kStatorTriggerCurrent, WRConsts.kStatorTriggerTime);
    ;

    wristConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    wristConfig.voltageCompSaturation = 12.0;

    wristConfig.motionCruiseVelocity = WRConsts.kMMVelocity;
    wristConfig.motionAcceleration = WRConsts.kMMAcceleration;
    wristConfig.motionCurveStrength = WRConsts.kMMSCurveStrength;

    wristConfig.reverseSoftLimitThreshold = Conversions.degreesToFalcon(WRConsts.kAngleMin, WRConsts.kGearRatio);
    wristConfig.reverseSoftLimitEnable = true;
    wristConfig.forwardSoftLimitThreshold = Conversions.degreesToFalcon(WRConsts.kAngleMax, WRConsts.kGearRatio);
    wristConfig.forwardSoftLimitEnable = true;

    wristConfig.neutralDeadband = WRConsts.kNeutralDeadband;

    wristConfig.slot0.allowableClosedloopError = WRConsts.kAllowedError;

    return wristConfig;
  }

  public static CANCoderConfiguration wristCancoderConfig( )
  {
    CANCoderConfiguration config = new CANCoderConfiguration( );
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.magnetOffsetDegrees = (Constants.isComp) ? WRConsts.kCompOffset : WRConsts.kBetaOffset;
    config.sensorDirection = true;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    return config;
  }

}
