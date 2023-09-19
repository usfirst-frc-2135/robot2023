package frc.robot.team1678.frc2022.drivers;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import frc.robot.lib.util.CTREConfigs6;
import frc.robot.lib.util.CTREModuleState;
import frc.robot.lib.util.SwerveModuleConstants;
import frc.robot.team2135.PhoenixUtil6;

public class SwerveModule
{
  public int                     m_moduleNumber;
  private double                 m_angleOffset;
  private TalonFX                m_angleMotor;
  private TalonFX                m_driveMotor;
  private CANcoder               m_angleEncoder;
  private double                 m_lastAngle;

  private SimpleMotorFeedforward m_feedforward            =
      new SimpleMotorFeedforward(SWConsts.driveKS, SWConsts.driveKV, SWConsts.driveKA);
  private DutyCycleOut           m_dutyCycleRequest       = new DutyCycleOut(0.0);
  private VelocityVoltage        m_velocityVoltageRequest = new VelocityVoltage(0.0);
  private PositionVoltage        m_positionVoltageRequest = new PositionVoltage(0.0);
  private Slot0Configs           m_slot0Config            = new Slot0Configs( );

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants)
  {
    StringBuilder driveName = new StringBuilder("drive" + moduleNumber);
    StringBuilder angleName = new StringBuilder("angle" + moduleNumber);

    this.m_moduleNumber = moduleNumber;
    m_angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    m_angleEncoder = new CANcoder(moduleConstants.cancoderID, Constants.Ports.kCANCarnivore);
    PhoenixUtil6.getInstance( ).canCoderInitialize6(m_angleEncoder, angleName.toString( ), CTREConfigs6.swerveCancoderConfig( ));

    /* Angle Motor Config */
    m_angleMotor = new TalonFX(moduleConstants.angleMotorID, Constants.Ports.kCANCarnivore);
    PhoenixUtil6.getInstance( ).talonFXInitialize6(m_angleMotor, angleName.toString( ), CTREConfigs6.swerveAngleFXConfig( ));

    TalonFXConfiguration angleConfig = CTREConfigs6.swerveAngleFXConfig( );
    m_slot0Config.kP = angleConfig.Slot0.kP;
    m_slot0Config.kI = angleConfig.Slot0.kI;
    m_slot0Config.kD = angleConfig.Slot0.kD;
    // TODO: Does the slot config need to be applied?

    /* Drive Motor Config */
    m_driveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.Ports.kCANCarnivore);
    PhoenixUtil6.getInstance( ).talonFXInitialize6(m_driveMotor, driveName.toString( ), CTREConfigs6.swerveDriveFXConfig( ));

    m_lastAngle = getState( ).angle.getDegrees( );
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
  {
    desiredState = CTREModuleState.optimize(desiredState, getState( ).angle); // Custom optimize command, since default
                                                                             // WPILib optimize assumes continuous
                                                                             // controller which CTRE is not

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

    double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SWConsts.maxSpeed * 0.01)) ? m_lastAngle
        : desiredState.angle.getDegrees( ); // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    m_angleMotor
        .setControl(m_positionVoltageRequest.withPosition(Conversions.degreesToInputRotations(angle, SWConsts.angleGearRatio)));
    m_lastAngle = angle;
  }

  public void resetToAbsolute( )
  {
    double absolutePosition =
        Conversions.degreesToInputRotations(getCanCoder( ).getDegrees( ) - m_angleOffset, SWConsts.angleGearRatio);
    m_angleMotor.setRotorPosition(absolutePosition);
  }

  public void updateAnglePID(double kP, double kI, double kD)
  {
    if ((m_slot0Config.kP != kP) || (m_slot0Config.kI != kI) || (m_slot0Config.kD != kD))
    {
      m_slot0Config.kP = kP;
      m_slot0Config.kI = kI;
      m_slot0Config.kD = kD;
      m_angleMotor.getConfigurator( ).apply(m_slot0Config);
    }
  }

  public double[ ] getAnglePIDValues( )
  {
    double[ ] values =
    {
        m_slot0Config.kP, m_slot0Config.kI, m_slot0Config.kD
    };
    return values;
  }

  public Rotation2d getCanCoder( )
  {
    return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition( ).getValue( ));
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
        .fromDegrees(Conversions.rotationsToOutputDegrees(m_angleMotor.getPosition( ).getValue( ), SWConsts.angleGearRatio));
    return new SwerveModuleState(velocity, angle);
  }

  public SwerveModulePosition getPosition( )
  {
    double distance = Conversions.rotationsToMeters(m_driveMotor.getPosition( ).getValue( ), SWConsts.wheelCircumference,
        SWConsts.driveGearRatio);
    Rotation2d angle = Rotation2d
        .fromDegrees(Conversions.rotationsToOutputDegrees(m_angleMotor.getPosition( ).getValue( ), SWConsts.angleGearRatio));
    return new SwerveModulePosition(distance, angle);
  }

}
