package frc.robot.lib.util;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SWConsts;
import frc.robot.lib.math.Conversions;

public class SwerveModule
{
  public int                     m_moduleNumber;
  private TalonFX                m_driveMotor;
  private TalonFX                m_steerMotor;
  private CANcoder               m_steerEncoder;
  private double                 m_steerOffset;
  private double                 m_lastAngle;

  private SimpleMotorFeedforward m_feedforward            =
      new SimpleMotorFeedforward(SWConsts.driveKS, SWConsts.driveKV, SWConsts.driveKA);
  private DutyCycleOut           m_dutyCycleRequest       = new DutyCycleOut(0.0);
  private VelocityVoltage        m_velocityVoltageRequest = new VelocityVoltage(0.0);
  private PositionVoltage        m_positionVoltageRequest = new PositionVoltage(0.0);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants)
  {
    StringBuilder driveName = new StringBuilder("drive" + moduleNumber);
    StringBuilder steerName = new StringBuilder("steer" + moduleNumber);

    this.m_moduleNumber = moduleNumber;
    m_steerOffset = moduleConstants.steerOffset;

    /* Angle Encoder Config */
    m_steerEncoder = new CANcoder(moduleConstants.cancoderID, Constants.Ports.kCANCarnivore);
    PhoenixUtil6.getInstance( ).canCoderInitialize6(m_steerEncoder, steerName.toString( ), CTREConfigs6.swerveCancoderConfig( ));

    /* Angle Motor Config */
    m_steerMotor = new TalonFX(moduleConstants.steerMotorID, Constants.Ports.kCANCarnivore);
    PhoenixUtil6.getInstance( ).talonFXInitialize6(m_steerMotor, steerName.toString( ), CTREConfigs6.swerveAngleFXConfig( ));

    /* Drive Motor Config */
    m_driveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.Ports.kCANCarnivore);
    PhoenixUtil6.getInstance( ).talonFXInitialize6(m_driveMotor, driveName.toString( ), CTREConfigs6.swerveDriveFXConfig( ));

    m_lastAngle = getState( ).angle.getDegrees( );
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
  {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not
    desiredState = CTREModuleState.optimize(desiredState, getState( ).angle);

    if (isOpenLoop)
    {
      double percentOutput = desiredState.speedMetersPerSecond / SWConsts.maxSpeed;
      m_driveMotor.setControl(m_dutyCycleRequest.withOutput(percentOutput));
    }
    else
    {
      double velocity =
          Conversions.MPSToRPS(desiredState.speedMetersPerSecond, SWConsts.wheelCircumference, SWConsts.driveGearRatio);
      m_driveMotor.setControl(m_velocityVoltageRequest.withVelocity(velocity)
          .withFeedForward(m_feedforward.calculate(desiredState.speedMetersPerSecond)));
    }

    // Prevent rotating module if speed is less then 1%. Prevents Jittering
    double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SWConsts.maxSpeed * 0.01)) ? m_lastAngle
        : desiredState.angle.getDegrees( );
    m_steerMotor
        .setControl(m_positionVoltageRequest.withPosition(Conversions.degreesToInputRotations(angle, SWConsts.steerGearRatio)));
    m_lastAngle = angle;
  }

  public void resetToAbsolute( )
  {
    m_steerMotor.setRotorPosition(getCanCoderRotations( ) - m_steerOffset);
  }

  public double getCanCoderRotations( )
  {
    return m_steerEncoder.getAbsolutePosition( ).refresh( ).getValue( );
  }

  public double getTargetAngle( )
  {
    return m_lastAngle;
  }

  public SwerveModuleState getState( )
  {
    double velocity =
        Conversions.RPSToMPS(m_driveMotor.getVelocity( ).getValue( ), SWConsts.wheelCircumference, SWConsts.driveGearRatio);
    Rotation2d angle = Rotation2d
        .fromDegrees(Conversions.rotationsToOutputDegrees(m_steerMotor.getPosition( ).getValue( ), SWConsts.steerGearRatio));
    return new SwerveModuleState(velocity, angle);
  }

  public SwerveModulePosition getPosition( )
  {
    double distance = Conversions.rotationsToMeters(m_driveMotor.getPosition( ).getValue( ), SWConsts.wheelCircumference,
        SWConsts.driveGearRatio);
    Rotation2d angle = Rotation2d
        .fromDegrees(Conversions.rotationsToOutputDegrees(m_steerMotor.getPosition( ).getValue( ), SWConsts.steerGearRatio));
    return new SwerveModulePosition(distance, angle);
  }

}
