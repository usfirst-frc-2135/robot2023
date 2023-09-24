package frc.robot.lib.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.EXConsts;
import frc.robot.Constants.SWConsts;
import frc.robot.Constants.WRConsts;
import frc.robot.lib.math.Conversions;

public final class CTREConfigs6
{
  // Swerve modules

  public static TalonFXConfiguration swerveDriveFXConfig( )
  {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration( );

    driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SWConsts.driveClosedLoopRamp;
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SWConsts.driveClosedLoopRamp;

    driveConfig.CurrentLimits.SupplyCurrentLimit = SWConsts.driveSupplyCurrentLimit;
    driveConfig.CurrentLimits.SupplyCurrentThreshold = SWConsts.driveSupplyCurrentThreshold;
    driveConfig.CurrentLimits.SupplyTimeThreshold = SWConsts.driveSupplyTimeThreshold;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = SWConsts.driveSupplyCurrentLimitEnable;

    // driveConfig.CurrentLimits.*
    // driveConfig.Feedback.*
    // driveConfig.HardwareLimitSwitch.*
    // driveConfig.MotionMagic.*

    // driveConfig.MotorOutput.DutyCycleNeutralDeadband
    driveConfig.MotorOutput.Inverted = SWConsts.driveMotorInvert;
    driveConfig.MotorOutput.NeutralMode = SWConsts.driveNeutralMode;

    driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SWConsts.driveOpenLoopRamp;
    driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SWConsts.driveOpenLoopRamp;

    driveConfig.Slot0.kS = SWConsts.driveFFKS;
    driveConfig.Slot0.kV = SWConsts.driveFFKV;
    driveConfig.Slot0.kP = SWConsts.driveKP;
    driveConfig.Slot0.kI = SWConsts.driveKI;
    driveConfig.Slot0.kD = SWConsts.driveKD;

    // driveConfig.SoftwareLimitSwitch.*

    return driveConfig;
  }

  public static TalonFXConfiguration swerveAngleFXConfig( )
  {
    TalonFXConfiguration angleConfig = new TalonFXConfiguration( );

    angleConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SWConsts.angleClosedLoopRamp;
    angleConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SWConsts.angleClosedLoopRamp;

    angleConfig.CurrentLimits.SupplyCurrentLimit = SWConsts.angleSupplyCurrentLimit;
    angleConfig.CurrentLimits.SupplyCurrentThreshold = SWConsts.angleSupplyCurrentThreshold;
    angleConfig.CurrentLimits.SupplyTimeThreshold = SWConsts.angleSupplyTimeThreshold;
    angleConfig.CurrentLimits.SupplyCurrentLimitEnable = SWConsts.angleSupplyCurrentLimitEnable;

    //  angleConfig.CurrentLimits.*
    //  angleConfig.Feedback.*
    //  angleConfig.HardwareLimitSwitch.*
    //  angleConfig.MotionMagic.*

    //  angleConfig.MotorOutput.DutyCycleNeutralDeadband
    angleConfig.MotorOutput.Inverted = SWConsts.angleMotorInvert;
    angleConfig.MotorOutput.NeutralMode = SWConsts.angleNeutralMode;

    angleConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SWConsts.angleOpenLoopRamp;
    angleConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SWConsts.angleOpenLoopRamp;

    angleConfig.Slot0.kS = SWConsts.angleFFKS;
    angleConfig.Slot0.kV = SWConsts.angleFFKV;
    angleConfig.Slot0.kP = SWConsts.angleKP;
    angleConfig.Slot0.kI = SWConsts.angleKI;
    angleConfig.Slot0.kD = SWConsts.angleKD;

    //  angleConfig.SoftwareLimitSwitch.*

    return angleConfig;
  }

  public static CANcoderConfiguration swerveCancoderConfig( )
  {
    CANcoderConfiguration config = new CANcoderConfiguration( );

    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = SWConsts.angleCanCoderInvert;
    // config.MagnetSensor.MagnetOffset 

    return config;
  }

  // Elbow

  public static TalonFXConfiguration elbowMotorFXConfig( )
  {
    TalonFXConfiguration elConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // elConfig.ClosedLoopGeneral.*
    // elConfig.ClosedLoopRamps.*

    // Current limit settings
    elConfig.CurrentLimits.SupplyCurrentLimit = ELConsts.kSupplyCurrentLimit;
    elConfig.CurrentLimits.SupplyCurrentThreshold = ELConsts.kSupplyTriggerCurrent;
    elConfig.CurrentLimits.SupplyTimeThreshold = ELConsts.kSupplyTriggerTime;
    elConfig.CurrentLimits.SupplyCurrentLimitEnable = ELConsts.kSupplyCurrentLimitEnable;

    elConfig.CurrentLimits.StatorCurrentLimit = ELConsts.kStatorCurrentLimit;
    elConfig.CurrentLimits.StatorCurrentLimitEnable = ELConsts.kStatorCurrentLimitEnable;

    // Feedback settings
    // elConfig.Feedback.*

    // Hardware limit switches
    // elConfig.HardwareLimitSwitch.*

    // Motion Magic settings
    elConfig.MotionMagic.MotionMagicCruiseVelocity = ELConsts.kMMVelocity;
    elConfig.MotionMagic.MotionMagicAcceleration = ELConsts.kMMAcceleration;
    elConfig.MotionMagic.MotionMagicJerk = ELConsts.kMMJerk;

    // Motor output settings
    elConfig.MotorOutput.DutyCycleNeutralDeadband = ELConsts.kNeutralDeadband;
    elConfig.MotorOutput.Inverted = ELConsts.kInvertMotor;
    elConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Closed loop settings

    // Open Loop settings
    elConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
    // elConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod
    elConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    // Slot settings
    elConfig.Slot0.kS = ELConsts.kS;
    elConfig.Slot0.kV = ELConsts.kV;
    elConfig.Slot0.kP = ELConsts.kPidKp;
    elConfig.Slot0.kI = ELConsts.kPidKi;
    elConfig.Slot0.kD = ELConsts.kPidKd;

    // Software limit switches
    elConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Conversions.degreesToInputRotations(ELConsts.kAngleMin, ELConsts.kGearRatio);
    elConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Conversions.degreesToInputRotations(ELConsts.kAngleMax, ELConsts.kGearRatio);
    elConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return elConfig;
  }

  public static CANcoderConfiguration elbowCancoderConfig( )
  {
    CANcoderConfiguration config = new CANcoderConfiguration( );
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = ELConsts.kSensorDirection;
    config.MagnetSensor.MagnetOffset = (Constants.isComp) ? ELConsts.kCompOffset : ELConsts.kBetaOffset;
    return config;
  }

  //wrist 

  public static TalonFXConfiguration wristAngleFXConfig( )
  {
    TalonFXConfiguration wrConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // wrConfig.ClosedLoopGeneral.*
    // wrConfig.ClosedLoopRamps.*

    // Current limit settings
    wrConfig.CurrentLimits.SupplyCurrentLimit = WRConsts.kSupplyCurrentLimit;
    wrConfig.CurrentLimits.SupplyCurrentThreshold = WRConsts.kSupplyTriggerCurrent;
    wrConfig.CurrentLimits.SupplyTimeThreshold = WRConsts.kSupplyTriggerTime;
    wrConfig.CurrentLimits.SupplyCurrentLimitEnable = WRConsts.kSupplyCurrentLimitEnable;

    wrConfig.CurrentLimits.StatorCurrentLimit = WRConsts.kStatorCurrentLimit;
    wrConfig.CurrentLimits.StatorCurrentLimitEnable = WRConsts.kStatorCurrentLimitEnable;

    // Feedback settings
    // wrConfig.Feedback.*

    // Hardware limit switches
    // wrConfig.HardwareLimitSwitch.*

    // Motion Magic settings
    wrConfig.MotionMagic.MotionMagicCruiseVelocity = WRConsts.kMMVelocity;
    wrConfig.MotionMagic.MotionMagicAcceleration = WRConsts.kMMAcceleration;
    wrConfig.MotionMagic.MotionMagicJerk = WRConsts.kMMSCurveStrength;

    // Motor output settings
    wrConfig.MotorOutput.DutyCycleNeutralDeadband = WRConsts.kNeutralDeadband;
    wrConfig.MotorOutput.Inverted = WRConsts.kInvertMotor;
    wrConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Closed loop settings

    // Open Loop settings
    wrConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
    // wrConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod
    wrConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    // Slot settings
    wrConfig.Slot0.kS = WRConsts.kS;
    wrConfig.Slot0.kV = WRConsts.kV;
    wrConfig.Slot0.kP = WRConsts.kPidKp;
    wrConfig.Slot0.kI = WRConsts.kPidKi;
    wrConfig.Slot0.kD = WRConsts.kPidKd;

    // Software limit switches
    wrConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Conversions.degreesToInputRotations(WRConsts.kAngleMin, WRConsts.kGearRatio);
    wrConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    wrConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Conversions.degreesToInputRotations(WRConsts.kAngleMax, WRConsts.kGearRatio);
    wrConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return wrConfig;
  }

  public static CANcoderConfiguration wristCancoderConfig( )
  {
    CANcoderConfiguration config = new CANcoderConfiguration( );
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = WRConsts.kSensorDirection;
    config.MagnetSensor.MagnetOffset = (Constants.isComp) ? WRConsts.kCompOffset : WRConsts.kBetaOffset;
    return config;
  }

  // Extension

  public static TalonFXConfiguration extensionLengthFXConfig( )
  {
    TalonFXConfiguration exConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // exConfig.ClosedLoopGeneral.*
    // exConfig.ClosedLoopRamps.*

    // Current limit settings
    exConfig.CurrentLimits.SupplyCurrentLimit = EXConsts.kSupplyCurrentLimit;
    exConfig.CurrentLimits.SupplyCurrentThreshold = EXConsts.kSupplyTriggerCurrent;
    exConfig.CurrentLimits.SupplyTimeThreshold = EXConsts.kSupplyTriggerTime;
    exConfig.CurrentLimits.SupplyCurrentLimitEnable = EXConsts.kSupplyCurrentLimitEnable;

    exConfig.CurrentLimits.StatorCurrentLimit = EXConsts.kStatorCurrentLimit;
    exConfig.CurrentLimits.StatorCurrentLimitEnable = EXConsts.kStatorCurrentLimitEnable;

    // Feedback settings
    // exConfig.Feedback.*

    // Hardware limit switches
    // exConfig.HardwareLimitSwitch.*

    // Motion Magic settings
    exConfig.MotionMagic.MotionMagicCruiseVelocity = EXConsts.kMMVelocity;
    exConfig.MotionMagic.MotionMagicAcceleration = EXConsts.kMMAcceleration;
    exConfig.MotionMagic.MotionMagicJerk = EXConsts.kMMJerk;

    // Motor output settings
    exConfig.MotorOutput.DutyCycleNeutralDeadband = EXConsts.kNeutralDeadband;
    exConfig.MotorOutput.Inverted = EXConsts.kInvertMotor;
    exConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Closed loop settings

    // Open Loop settings
    exConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
    // exConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod
    exConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    // Slot settings
    exConfig.Slot0.kS = EXConsts.kS;
    exConfig.Slot0.kV = EXConsts.kV;
    exConfig.Slot0.kP = EXConsts.kPidKp;
    exConfig.Slot0.kI = EXConsts.kPidKi;
    exConfig.Slot0.kD = EXConsts.kPidKd;

    // Software limit switches
    exConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Conversions.inchesToWinchRotations(EXConsts.kLengthMin, EXConsts.kRolloutRatio);
    exConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    exConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Conversions.inchesToWinchRotations(EXConsts.kLengthMax, EXConsts.kRolloutRatio);
    exConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return exConfig;
  }
}
