// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ARMConsts;
import frc.robot.Constants.ARMConsts.ARMMode;

public class Arm extends SubsystemBase
{
  // Constants
  private static final int  CANTIMEOUT        = 30;  // CAN timeout in msec
  private static final int  PIDINDEX          = 0;   // PID in use (0-primary, 1-aux)
  private static final int  SLOTINDEX         = 0;   // Use first PID slot

  private final WPI_TalonFX m_Arm14           = new WPI_TalonFX(14);;
  private final WPI_TalonFX m_Arm15           = new WPI_TalonFX(15);;

  private boolean           m_validARM14;               // Health indicator for climber Talon 14
  private boolean           m_validARM15;               // Health indicator for climber Talon 15

  // Declare module variables
  private int               m_velocity        = ARMConsts.kMMVelocity;        // motion magic velocity
  private int               m_acceleration    = ARMConsts.kMMAcceleration;    // motion magic acceleration
  private int               m_sCurveStrength  = ARMConsts.kMMSCurveStrength;  // motion magic S curve smoothing
  private double            m_pidKf           = ARMConsts.kARMPidKf;           // PID force constant
  private double            m_pidKp           = ARMConsts.kARMPidKp;           // PID proportional
  private double            m_pidKi           = ARMConsts.kARMPidKi;           // PID integral
  private double            m_pidKd           = ARMConsts.kARMPidKd;           // PID derivative
  private int               m_ARMAllowedError = ARMConsts.kARMAllowedError;    // PID allowable closed loop error
  private double            m_toleranceInches = ARMConsts.kARMToleranceInches; // PID tolerance in inches

  private double            m_stowHeight      = ARMConsts.kStowHeight;         // Stow height

  private double            m_stickDeadband   = ARMConsts.kStickDeadband;      // joystick deadband
  private ARMMode           m_mode            = ARMMode.ARM_INIT;          // Mode active with joysticks

  private int               m_climberDebug    = 1; // DEBUG flag to disable/enable extra logging calls

  private boolean           m_calibrated      = false;  // Indicates whether the climber has been calibrated
  private double            m_targetDegrees   = 0.0;    // Target height in inches requested
  private double            m_curDegrees      = 0.0;    // Current climber height in inches
  private int               m_withinTolerance = 0;      // Counter for consecutive readings within tolerance

  private Timer             m_safetyTimer     = new Timer( ); // Safety timer for use in climber
  private double            m_safetyTimeout;                // Seconds that the timer ran before stopping
  private Timer             timer             = new Timer( );

  /**
   *
   */
  /** Creates a new ExampleSubsystem. */
  public Arm( )
  {
    setName("Arm");
    setSubsystem("Arm");

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand( )
  {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(( ) ->
    {
      /* one-time action goes here */
    });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition( )
  {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    // This method will be called once per scheduler run

    // if disabled, set LED when down

    if (m_validARM14)
    {
      int curCounts = (int) m_Arm14.getSelectedSensorPosition(0);
      m_curDegrees = countsToDegrees(curCounts);
      SmartDashboard.putNumber("ARM_curDegrees", m_curDegrees);
    }
    if (m_validARM15)
    {
      int curCounts = (int) m_Arm15.getSelectedSensorPosition(0);
      m_curDegrees = countsToDegrees(curCounts);
      SmartDashboard.putNumber("ARM_curDegrees", m_curDegrees);
    }
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
  }

  public void initialize( )
  {
    double curCounts = 0.0;

    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");

    setARMStopped( );

    if (m_validARM14)
      curCounts = m_Arm14.getSelectedSensorPosition(0);
    if (m_validARM14)
      curCounts = m_Arm15.getSelectedSensorPosition(0);

    m_curDegrees = countsToDegrees((int) curCounts);
    m_targetDegrees = m_curDegrees;
    DataLogManager.log(getSubsystem( ) + ": Init Target Inches: " + m_targetDegrees);
  }

  private int degreesToCounts(double degrees)
  {
    return (int) (degrees / ARMConsts.kInchesPerCount);
  }

  private double countsToDegrees(int counts)
  {
    return counts * ARMConsts.kInchesPerCount;
  }

  private int metersToNativeUnits(double meters)
  {
    return (int) (meters / ARMConsts.kMetersPerCount);
  }

  private double nativeUnitsToMeters(int counts)
  {
    return counts * ARMConsts.kMetersPerCount;
  }

  private void ARMTalonInitialize(WPI_TalonFX motor, boolean inverted)
  {}

  public void FollowerInitialize( )
  {
    if (m_validARM15)
    {
    }
  }

  public void moveELBOWWithJoysticks(XboxController joystick)
  {
    double yELBOWValue = 0.0;
    double motorOutput = 0.0;
    double manualSpeedMax = ARMConsts.kSpeedMaxManual;

    yELBOWValue = -joystick.getLeftY( );
    if (yELBOWValue > -m_stickDeadband && yELBOWValue < m_stickDeadband)
    {
      if (m_mode != ARMMode.ARM_STOPPED)
        DataLogManager.log(getSubsystem( ) + "ARM Stopped");
      m_mode = ARMMode.ARM_STOPPED;
    }
    else
    {
      // If joystick is above a value, ARM will move up
      if (yELBOWValue > m_stickDeadband)
      {
        if (m_mode != ARMMode.ARM_UP)
          DataLogManager.log(getSubsystem( ) + "ARM Up");
        m_mode = ARMMode.ARM_UP;

        yELBOWValue -= m_stickDeadband;
        yELBOWValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yELBOWValue * Math.abs(yELBOWValue));
      }
      // If joystick is below a value, ARM will move down
      else if (yELBOWValue < -m_stickDeadband)
      {
        if (m_mode != ARMMode.ARM_DOWN)
          DataLogManager.log(getSubsystem( ) + "ARM Down");
        m_mode = ARMMode.ARM_DOWN;

        yELBOWValue += m_stickDeadband;
        yELBOWValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yELBOWValue * Math.abs(yELBOWValue));
      }
    }
  }

  public void moveWRISTWithJoysticks(XboxController joystick)
  {
    double yWRISTValue = 0.0;
    double motorOutput = 0.0;
    double manualSpeedMax = ARMConsts.kSpeedMaxManual;

    yWRISTValue = -joystick.getRightY( );
    if (yWRISTValue > -m_stickDeadband && yWRISTValue < m_stickDeadband)
    {
      if (m_mode != ARMMode.ARM_STOPPED)
        DataLogManager.log(getSubsystem( ) + "WRIST Stopped");
      m_mode = ARMMode.ARM_STOPPED;
    }
    else
    {
      // If joystick is above a value, WRIST will move up
      if (yWRISTValue > m_stickDeadband)
      {
        if (m_mode != ARMMode.ARM_UP)
          DataLogManager.log(getSubsystem( ) + "WRIST Up");
        m_mode = ARMMode.ARM_UP;

        yWRISTValue -= m_stickDeadband;
        yWRISTValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yWRISTValue * Math.abs(yWRISTValue));
      }
      // If joystick is below a value, WRIST will move down
      else if (yWRISTValue < -m_stickDeadband)
      {
        if (m_mode != ARMMode.ARM_DOWN)
          DataLogManager.log(getSubsystem( ) + "WRIST Down");
        m_mode = ARMMode.ARM_DOWN;

        yWRISTValue += m_stickDeadband;
        yWRISTValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yWRISTValue * Math.abs(yWRISTValue));
      }
    }
  }

  public void setARMStopped( )
  {
    DataLogManager.log(getSubsystem( ) + ": ARM Set Arm Stopped");

    if (m_validARM14)
      m_Arm14.set(ControlMode.PercentOutput, 0.0);

    if (m_validARM15)
      m_Arm15.set(ControlMode.PercentOutput, 0.0);
  }
}
