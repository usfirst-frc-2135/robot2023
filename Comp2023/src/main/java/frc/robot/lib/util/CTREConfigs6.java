package frc.robot.lib.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.EXConsts;
import frc.robot.Constants.WRConsts;
import frc.robot.lib.math.Conversions;

public final class CTREConfigs6
{
  // Elbow

  public static TalonFXConfiguration elbowMotorFXConfig( )
  {
    TalonFXConfiguration elConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // elConfig.ClosedLoopGeneral.*
    // elConfig.ClosedLoopRamps.*

    // Current limit settings
    elConfig.CurrentLimits.SupplyCurrentLimit = Constants.ELConsts.kSupplyCurrentLimit;
    elConfig.CurrentLimits.SupplyCurrentThreshold = Constants.ELConsts.kSupplyTriggerCurrent;
    elConfig.CurrentLimits.SupplyTimeThreshold = Constants.ELConsts.kSupplyTriggerTime;
    elConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.ELConsts.kSupplyCurrentLimitEnable;

    elConfig.CurrentLimits.StatorCurrentLimit = ELConsts.kStatorCurrentLimit;
    elConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.ELConsts.kStatorCurrentLimitEnable;

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
    elConfig.Slot0.kS = Constants.ELConsts.kS;
    elConfig.Slot0.kV = Constants.ELConsts.kV;
    elConfig.Slot0.kP = Constants.ELConsts.kPidKp;
    elConfig.Slot0.kI = Constants.ELConsts.kPidKi;
    elConfig.Slot0.kD = Constants.ELConsts.kPidKd;

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
    wrConfig.CurrentLimits.SupplyCurrentLimit = Constants.WRConsts.kSupplyCurrentLimit;
    wrConfig.CurrentLimits.SupplyCurrentThreshold = Constants.WRConsts.kSupplyTriggerCurrent;
    wrConfig.CurrentLimits.SupplyTimeThreshold = Constants.WRConsts.kSupplyTriggerTime;
    // wrConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.WRConsts.kSupplyCurrentLimitEnable;

    wrConfig.CurrentLimits.StatorCurrentLimit = WRConsts.kStatorCurrentLimit;
    // wrConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.WRConsts.kStatorCurrentLimitEnable;

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
    //TODO - implement a kS ?
    wrConfig.Slot0.kV = Constants.WRConsts.kV;
    wrConfig.Slot0.kP = Constants.WRConsts.kPidKp;
    wrConfig.Slot0.kI = Constants.WRConsts.kPidKi;
    wrConfig.Slot0.kD = Constants.WRConsts.kPidKd;

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
    exConfig.CurrentLimits.SupplyCurrentLimit = Constants.EXConsts.kSupplyCurrentLimit;
    exConfig.CurrentLimits.SupplyCurrentThreshold = Constants.EXConsts.kSupplyTriggerCurrent;
    exConfig.CurrentLimits.SupplyTimeThreshold = Constants.EXConsts.kSupplyTriggerTime;
    // exConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.EXConsts.kSupplyCurrentLimitEnable;

    exConfig.CurrentLimits.StatorCurrentLimit = EXConsts.kStatorCurrentLimit;
    // exConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.EXConsts.kStatorCurrentLimitEnable;

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
    // exConfig.Slot0.kS = Constants.EXConsts.kS;
    exConfig.Slot0.kV = Constants.EXConsts.kV;
    exConfig.Slot0.kP = Constants.EXConsts.kPidKp;
    exConfig.Slot0.kI = Constants.EXConsts.kPidKi;
    exConfig.Slot0.kD = Constants.EXConsts.kPidKd;

    // Software limit switches
    exConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Conversions.degreesToInputRotations(ELConsts.kAngleMin, ELConsts.kGearRatio);
    exConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    exConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Conversions.degreesToInputRotations(ELConsts.kAngleMax, ELConsts.kGearRatio);
    exConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return exConfig;
  }
}
