package frc.robot.lib.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.ELConsts;
import frc.robot.lib.math.Conversions;

public final class CTREConfigs6
{
  // Elbow

  public static TalonFXConfiguration elbowMotorFXConfig( )
  {
    TalonFXConfiguration elbowConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // elbowConfig.ClosedLoopGeneral.*
    // elbowConfig.ClosedLoopRamps.*

    // Current limit settings
    elbowConfig.CurrentLimits.SupplyCurrentLimit = Constants.ELConsts.kSupplyCurrentLimit;
    elbowConfig.CurrentLimits.SupplyCurrentThreshold = Constants.ELConsts.kSupplyTriggerCurrent;
    elbowConfig.CurrentLimits.SupplyTimeThreshold = Constants.ELConsts.kSupplyTriggerTime;
    elbowConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.ELConsts.kSupplyCurrentLimitEnable;

    elbowConfig.CurrentLimits.StatorCurrentLimit = ELConsts.kStatorCurrentLimit;
    elbowConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.ELConsts.kStatorCurrentLimitEnable;

    // Feedback settings
    // elbowConfig.Feedback.*

    // Hardware limit switches
    // elbowConfig.HardwareLimitSwitch.*

    // Motion Magic settings
    elbowConfig.MotionMagic.MotionMagicCruiseVelocity = ELConsts.kMMVelocity;
    elbowConfig.MotionMagic.MotionMagicAcceleration = ELConsts.kMMAcceleration;
    elbowConfig.MotionMagic.MotionMagicJerk = ELConsts.kMMJerk;

    // Motor output settings
    elbowConfig.MotorOutput.DutyCycleNeutralDeadband = ELConsts.kNeutralDeadband;
    elbowConfig.MotorOutput.Inverted = ELConsts.kInvertMotor;
    elbowConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Closed loop settings

    // Open Loop settings
    elbowConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
    // elbowConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod
    elbowConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    // Slot settings
    elbowConfig.Slot0.kS = Constants.ELConsts.kS;
    elbowConfig.Slot0.kV = Constants.ELConsts.kV;
    elbowConfig.Slot0.kP = Constants.ELConsts.kPidKp;
    elbowConfig.Slot0.kI = Constants.ELConsts.kPidKi;
    elbowConfig.Slot0.kD = Constants.ELConsts.kPidKd;

    // Software limit switches
    elbowConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Conversions.degreesToRotations(ELConsts.kAngleMin, ELConsts.kGearRatio);
    elbowConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elbowConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Conversions.degreesToRotations(ELConsts.kAngleMax, ELConsts.kGearRatio);
    elbowConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return elbowConfig;
  }

  public static CANcoderConfiguration elbowCancoderConfig( )
  {
    CANcoderConfiguration config = new CANcoderConfiguration( );
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = ELConsts.kSensorDirection;
    config.MagnetSensor.MagnetOffset = (Constants.isComp) ? ELConsts.kCompOffset : ELConsts.kBetaOffset;
    return config;
  }
}
