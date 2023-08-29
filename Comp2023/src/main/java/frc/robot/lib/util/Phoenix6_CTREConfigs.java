package frc.robot.lib.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.ELConsts;
import frc.robot.lib.math.Conversions;

public final class Phoenix6_CTREConfigs
{
  // Elbow

  public static TalonFXConfiguration elbowAngleFXConfig( )
  {
    TalonFXConfiguration elbowConfig = new TalonFXConfiguration( );

    elbowConfig.Slot0.kP = Constants.ELConsts.kPidKp;
    elbowConfig.Slot0.kI = Constants.ELConsts.kPidKi;
    elbowConfig.Slot0.kD = Constants.ELConsts.kPidKd;
    elbowConfig.Slot0.kV = Constants.ELConsts.kPidKf;

    elbowConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elbowConfig.CurrentLimits.SupplyCurrentLimit = Constants.ELConsts.kSupplyCurrentLimit;
    elbowConfig.CurrentLimits.SupplyCurrentThreshold = Constants.ELConsts.kSupplyTriggerCurrent;
    elbowConfig.CurrentLimits.SupplyTimeThreshold = Constants.ELConsts.kSupplyTriggerTime;

    elbowConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elbowConfig.CurrentLimits.StatorCurrentLimit = ELConsts.kStatorCurrentLimit;

    elbowConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Conversions.degreesToFalcon(ELConsts.kAngleMin, ELConsts.kGearRatio);
    elbowConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elbowConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Conversions.degreesToFalcon(ELConsts.kAngleMax, ELConsts.kGearRatio);
    elbowConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    //elbowConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    //elbowConfig.voltageCompSaturation = 12.0;

    elbowConfig.MotionMagic.MotionMagicCruiseVelocity = ELConsts.kMMVelocity;
    elbowConfig.MotionMagic.MotionMagicAcceleration = ELConsts.kMMAcceleration;
    elbowConfig.MotionMagic.MotionMagicJerk = ELConsts.kMMSCurveStrength;

    //elbowConfig.slot0.allowableClosedloopError = ELConsts.kAllowedError;

    elbowConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elbowConfig.MotorOutput.Inverted = ELConsts.kInvertMotor;

    elbowConfig.MotorOutput.DutyCycleNeutralDeadband = ELConsts.kNeutralDeadband;

    return elbowConfig;
  }

}
