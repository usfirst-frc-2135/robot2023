package frc.robot.lib.util;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants;
import frc.robot.Constants.SWConsts;
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
    SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(SWConsts.driveEnableCurrentLimit,
        SWConsts.driveContinuousCurrentLimit, SWConsts.drivePeakCurrentLimit, SWConsts.drivePeakCurrentDuration);

    config.slot0.kP = SWConsts.driveKP;
    config.slot0.kI = SWConsts.driveKI;
    config.slot0.kD = SWConsts.driveKD;
    config.slot0.kF = SWConsts.driveKF;
    config.supplyCurrLimit = driveSupplyLimit;
    config.initializationStrategy = SensorInitializationStrategy.BootToZero;
    config.openloopRamp = SWConsts.openLoopRamp;
    config.closedloopRamp = SWConsts.closedLoopRamp;
    return config;
  }

  public static TalonFXConfiguration swerveAngleFXConfig( )
  {
    TalonFXConfiguration angleConfig = new TalonFXConfiguration( );
    SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(SWConsts.angleEnableCurrentLimit,
        SWConsts.angleContinuousCurrentLimit, SWConsts.anglePeakCurrentLimit, SWConsts.anglePeakCurrentDuration);

    angleConfig.slot0.kP = SWConsts.angleKP;
    angleConfig.slot0.kI = SWConsts.angleKI;
    angleConfig.slot0.kD = SWConsts.angleKD;
    angleConfig.slot0.kF = SWConsts.angleKF;
    angleConfig.supplyCurrLimit = angleSupplyLimit;
    angleConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    return angleConfig;
  }

  public static CANCoderConfiguration swerveCancoderConfig( )
  {
    CANCoderConfiguration config = new CANCoderConfiguration( );
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.sensorDirection = SWConsts.canCoderInvert;
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
   * elbowConfig.Slot0.kP = ELConsts.kPidKp;
   * elbowConfig.Slot0.kI = ELConsts.kPidKi;
   * elbowConfig.Slot0.kD = ELConsts.kPidKd;
   * elbowConfig.Slot0.kV = ELConsts.kPidKf;
   * 
   * elbowConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
   * elbowConfig.CurrentLimits.SupplyCurrentLimit = ELConsts.kSupplyCurrentLimit;
   * elbowConfig.CurrentLimits.SupplyCurrentThreshold = ELConsts.kSupplyTriggerCurrent;
   * elbowConfig.CurrentLimits.SupplyTimeThreshold = ELConsts.kSupplyTriggerTime;
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
  //   config.sensorDirection = ELConsts.kInvertCANCoder;
  //   config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
  //   config.sensorTimeBase = SensorTimeBase.PerSecond;
  //   return config;
  // }

  // Extension

  public static TalonFXConfiguration extensionLengthFXConfig( )
  {
    TalonFXConfiguration extensionConfig = new TalonFXConfiguration( );

    extensionConfig.slot0.kP = EXConsts.kPidKp;
    extensionConfig.slot0.kI = EXConsts.kPidKi;
    extensionConfig.slot0.kD = EXConsts.kPidKd;
    extensionConfig.slot0.kF = EXConsts.kV;

    extensionConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, EXConsts.kSupplyCurrentLimit,
        EXConsts.kSupplyTriggerCurrent, EXConsts.kSupplyTriggerTime);
    extensionConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, EXConsts.kStatorCurrentLimit,
        EXConsts.kStatorTriggerCurrent, EXConsts.kStatorTriggerTime);

    extensionConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    extensionConfig.voltageCompSaturation = 12.0;

    extensionConfig.motionCruiseVelocity = EXConsts.kMMVelocity;
    extensionConfig.motionAcceleration = EXConsts.kMMAcceleration;
    extensionConfig.motionCurveStrength = 1;

    extensionConfig.neutralDeadband = EXConsts.kNeutralDeadband;

    extensionConfig.slot0.allowableClosedloopError = EXConsts.kAllowedError;

    return extensionConfig;
  }

  // Wrist

  public static TalonFXConfiguration wristAngleFXConfig( )
  {
    TalonFXConfiguration wristConfig = new TalonFXConfiguration( );

    wristConfig.slot0.kP = WRConsts.kPidKp;
    wristConfig.slot0.kI = WRConsts.kPidKi;
    wristConfig.slot0.kD = WRConsts.kPidKd;
    wristConfig.slot0.kF = WRConsts.kV;

    wristConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, WRConsts.kSupplyCurrentLimit,
        WRConsts.kSupplyTriggerCurrent, WRConsts.kSupplyTriggerTime);
    wristConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, WRConsts.kStatorCurrentLimit,
        WRConsts.kStatorTriggerCurrent, WRConsts.kStatorTriggerTime);
    ;

    wristConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    wristConfig.voltageCompSaturation = 12.0;

    wristConfig.motionCruiseVelocity = WRConsts.kMMVelocity;
    wristConfig.motionAcceleration = WRConsts.kMMAcceleration;
    wristConfig.motionCurveStrength = 1;

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
