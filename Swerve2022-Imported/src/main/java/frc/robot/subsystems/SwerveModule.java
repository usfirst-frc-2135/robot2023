// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;

public class SwerveModule
{
  private static final double          kWheelRadius                  = 0.0508;
  private static final int             kEncoderResolution            = 4096;

  private static final double          kModuleMaxAngularVelocity     = Math.PI;       //based off of Drivetrain.kMaxAngularSpeed
  private static final double          kModuleMaxAngularAcceleration = 2 * Math.PI;   // radians per second squared

  public final WPI_TalonFX             m_driveMotor;
  public final WPI_TalonFX             m_turningMotor;

  public final CANCoder                m_turningCANCoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController          m_drivePIDController          = new PIDController(1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController  m_turningPIDController        = new ProfiledPIDController(1, 0, 0,
      new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward            = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward             = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   */
  public SwerveModule(int driveMotorCANID, int turningMotorCANID, int addressCANID)
  {
    m_driveMotor = new WPI_TalonFX(driveMotorCANID, DTConsts.kCANBusString);
    m_turningMotor = new WPI_TalonFX(turningMotorCANID, DTConsts.kCANBusString);

    m_turningCANCoder = new CANCoder(addressCANID, DTConsts.kCANBusString);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState( )
  {
    return new SwerveModuleState(getSpeedMPS(m_driveMotor), new Rotation2d(degToRad(m_turningCANCoder)));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState
   *          Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState)
  {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(degToRad(m_turningCANCoder)));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(getSpeedMPS(m_driveMotor), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(degToRad(m_turningCANCoder), state.angle.getRadians( ));

    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint( ).velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }

  public double getDistanceMeters(WPI_TalonFX driveMotor)
  {
    return nativeUnitsToMeters(driveMotor.getSelectedSensorPosition( ));
  }

  public void resetEncoders( )
  {
    m_turningMotor.configFactoryDefault( );
    m_turningCANCoder.configAllSettings(RobotContainer.swerveCancoderConfig( ));
  }

  private double getSpeedMPS(WPI_TalonFX driveMotor)
  {
    return nativeUnitsToMPS(driveMotor.getSelectedSensorVelocity( ));
  }

  private double nativeUnitsToMeters(double nativeUnits)
  {
    return nativeUnits * DTConsts.kEncoderMetersPerCount;
  }

  private double nativeUnitsToMPS(double nativeUnitsVelocity)
  {
    return nativeUnitsVelocity * DTConsts.kEncoderMetersPerCount * 10;
  }

  private double degToRad(CANCoder encoder)
  {
    return Units.degreesToRadians(encoder.getAbsolutePosition( ));
  }
}
