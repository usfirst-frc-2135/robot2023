// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A simplified stub class that simulates the API of a common "smart" motor controller.
 *
 * <p>
 * Wrapper for a TalonSRX with integrated encoder
 */
public class ExampleSmartMotorController implements MotorController
{
  public enum PIDMode
  {
    kPosition, kVelocity, kMovementWitchcraft
  }

  private static final double   kEncoderCPR         = 4096;

  // private final static double                       ks            = 0.0;   // Volts to overcome static friction
  // private final static double                       kf            = 0.0;   // Volts per velocity unit
  private final static double   kp                  = 0.4;   // (native units) 10% * 102.3 / 1023
  private final static double   ki                  = 0.0;
  private final static double   kd                  = 0.0;

  private static final double   kGearRatio          = 9.0;                                // TODO: Change to actual value
  private static final double   kForearmMassKg      = 2.0;
  private static final double   kDrumDiameterMeters = Units.inchesToMeters(1.375); // Drum diameter in meters
  private static final double   kLengthMeters       = Units.inchesToMeters(60.0);  // Maximum length in meters
  private static final double   kDrumCircumMeters   = kDrumDiameterMeters * Math.PI;      // Drum diameter in meters
  // private static final double   kRolloutRatioMeters = kDrumCircumMeters / kGearRatio;     // Meters per shaft rotation

  private WPI_TalonSRX          m_motor;
  private TalonSRXSimCollection m_motorSim;
  private final ElevatorSim     m_armSim            = new ElevatorSim(DCMotor.getVex775Pro(1), kGearRatio, kForearmMassKg,
      kDrumDiameterMeters / 2, -kLengthMeters, kLengthMeters, false);

  /**
   * Creates a new ExampleSmartMotorController.
   *
   * @param port
   *          The port for the controller.
   */
  @SuppressWarnings("PMD.UnusedFormalParameter")
  public ExampleSmartMotorController(int port)
  {
    // Create the Talon SRX object and the attached CTRE Mag encoder
    m_motor = new WPI_TalonSRX(port);
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_motor.selectProfileSlot(0, 0);

    m_motor.config_kP(0, kp);
    m_motor.config_kI(0, ki);
    m_motor.config_kD(0, kd);

    // Connect simulation object to the Talon SRX
    m_motorSim = m_motor.getSimCollection( );
  }

  /**
   * Periodic processing for this motor/controller
   * 
   */
  public void periodic( )
  {
    // This method will be called once per scheduler run

    // Set input motor voltage from the motor setting
    m_motorSim.setBusVoltage(RobotController.getInputVoltage( ));
    m_armSim.setInput(m_motorSim.getMotorOutputLeadVoltage( ));

    // update for 20 msec loop
    m_armSim.update(0.020);

    // // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_motorSim.setQuadratureRawPosition((int) (kEncoderCPR * m_armSim.getPositionMeters( ) / kDrumCircumMeters));
    m_motorSim.setQuadratureVelocity((int) (kEncoderCPR * (m_armSim.getVelocityMetersPerSecond( ) / kDrumCircumMeters) / 10));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps( )));

    SmartDashboard.putNumber("2-motorVolts", m_motorSim.getMotorOutputLeadVoltage( ));    // Voltage applied to motor
    SmartDashboard.putNumber("3-armPos", m_armSim.getPositionMeters( ));                  // Get elevator position from simulation (meters)
    SmartDashboard.putNumber("4-armVel", m_armSim.getVelocityMetersPerSecond( ));         // Get elevator velocity from simulation (mps)
    SmartDashboard.putNumber("5-Rotations", getEncoderPosition( ));                       // Output shaft distance (rotations)
    SmartDashboard.putNumber("6-Velocity", getEncoderVelocity( ));                            // Output shaft velocity (rps)
    SmartDashboard.putNumber("7-normError", m_motor.getClosedLoopError( ) / kEncoderCPR); // Normalized error in shaft rotations 
  }

  /**
   * Converts shaft rotations to encoder counts
   *
   * @param rotation
   *          The rotation value to be converted.
   * @return encoderCounts
   */
  private double rotationsToCounts(double rotation)
  {
    return rotation * kEncoderCPR;
  }

  /**
   * Converts encoder counts to shaft rotations
   *
   * @param encoderCounts
   *          The count value to be converted.
   * @return rotations
   */
  private double countsToRotations(double encoderCounts)
  {
    return encoderCounts / kEncoderCPR;
  }

  /**
   * Set the setpoint of the smart controller in PID mode.
   *
   * @param mode
   *          The mode of the PID controller.
   * @param setpoint
   *          The controller setpoint.
   * @param arbFeedforward
   *          An arbitrary feedforward output (from -1 to 1).
   */
  public void setSetpoint(PIDMode mode, double setpoint, double arbFeedforward)
  {
    ControlMode controlMode;

    switch (mode)
    {
      default :
      case kPosition :  // Position PID
        controlMode = ControlMode.Position;
        break;

      case kVelocity :  // Velocity PID
        controlMode = ControlMode.Velocity;
        setpoint /= 10; // Adjust for CTRE units
        break;

      case kMovementWitchcraft :  // Motion Magic profile TODO: not yet implemented
        controlMode = ControlMode.MotionMagic;
        break;
    }

    // Talon SRX is before gearbox, but CTRE mag encoder is after it (multiply by gear ratio)
    m_motor.set(controlMode, rotationsToCounts(setpoint) * kGearRatio);
  }

  /**
   * Places this motor controller in follower mode.
   *
   * @param leader
   *          The leader to follow.
   */
  public void follow(ExampleSmartMotorController leader)
  {}

  /**
   * Returns the encoder position in rotations.
   *
   * @return The current encoder position in rotations.
   */
  public double getEncoderPosition( )
  {
    return countsToRotations(m_motor.getSelectedSensorPosition(0));
  }

  /**
   * Returns the encoder velocity.
   *
   * @return The current encoder velocity in rotations per second.
   */
  public double getEncoderVelocity( )
  {
    return countsToRotations(m_motor.getSelectedSensorVelocity(0) * 10);
  }

  /**
   * Resets the encoder to zero distance.
   * 
   */
  public void resetEncoder( )
  {
    m_motor.setSelectedSensorPosition(0, 0, 0);
  }

  /**
   * Set motor to a known percent output value.
   *
   * @param percentOutput
   *          Percent output in range [-1.0, 1.0].
   */
  @Override
  public void set(double percentOutput)
  {
    m_motor.set(ControlMode.PercentOutput, percentOutput);
  }

  /**
   * Returns the motor setting in use.
   *
   * @return percentOutput in range [-1.0, 1.0].
   */
  @Override
  public double get( )
  {
    return m_motor.getMotorOutputPercent( );
  }

  /**
   * Set motor inverted state.
   *
   * @param isInverted
   *          Invert motor if true
   */
  @Override
  public void setInverted(boolean isInverted)
  {
    m_motor.setInverted(isInverted);
  }

  /**
   * Get motor inverted state.
   *
   * @return invertedState
   */
  @Override
  public boolean getInverted( )
  {
    return m_motor.getInverted( );
  }

  /**
   * Set motor to the disabled state.
   * 
   */
  @Override
  public void disable( )
  {
    m_motor.set(ControlMode.Disabled, 0);
  }

  /**
   * Set motor to the stopped state.
   * 
   */
  @Override
  public void stopMotor( )
  {
    set(0.0);
  }

}
