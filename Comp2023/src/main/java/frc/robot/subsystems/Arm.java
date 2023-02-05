// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ARMConsts;
import frc.robot.Constants.ARMConsts.ARMMode;
import frc.robot.team2135.PhoenixUtil;

public class Arm extends SubsystemBase
{
  // Constants
  private static final int  CANTIMEOUT        = 30;  // CAN timeout in msec
  private static final int  PIDINDEX          = 0;   // PID in use (0-primary, 1-aux)
  private static final int  SLOTINDEX         = 0;   // Use first PID slot

  private final WPI_TalonFX m_Arm14           = new WPI_TalonFX(14);  //elbow
  private final WPI_TalonFX m_Arm15           = new WPI_TalonFX(15);  //wrist

  private boolean           m_validEL14;               // Health indicator for climber Talon 14
  private boolean           m_validWR15;               // Health indicator for climber Talon 15

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
  private double            m_targetELDegrees = 0.0;    // Target angle in degrees
  private double            m_curELDegrees    = 0.0;    // Current angle in degrees
  private double            m_targetWRDegrees = 0.0;    // Target angle in degrees
  private double            m_curWRDegrees    = 0.0;    // Current angle in degrees
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

    m_validEL14 = PhoenixUtil.getInstance( ).talonFXInitialize(m_Arm14, "elbow14");
    m_validWR15 = PhoenixUtil.getInstance( ).talonFXInitialize(m_Arm15, "wrist15");

    // SmartDashboard.putBoolean("HL_validCL14", m_validCL14);
    // SmartDashboard.putBoolean("HL_validCL15", m_validCL15);

    // // Check if solenoids are functional or blacklisted
    // DataLogManager.log(getSubsystem( ) + ": CL Climber Solenoid is " + ((m_gateHook.isDisabled( )) ? "BLACKLISTED" : "OK"));

    // // Initialize Variables
    // SmartDashboard.putNumber("CL_velocity", m_velocity);
    // SmartDashboard.putNumber("CL_acceleration", m_acceleration);
    // SmartDashboard.putNumber("CL_sCurveStrength", m_sCurveStrength);
    // SmartDashboard.putNumber("CL_pidKf", m_pidKf);
    // SmartDashboard.putNumber("CL_pidKp", m_pidKp);
    // SmartDashboard.putNumber("CL_pidKi", m_pidKi);
    // SmartDashboard.putNumber("CL_pidKd", m_pidKd);

    // SmartDashboard.putNumber("CL_stowHeight", m_stowHeight);
    // SmartDashboard.putNumber("CL_extendL2", m_extendL2);
    // SmartDashboard.putNumber("CL_rotateL3", m_rotateL3);
    // SmartDashboard.putNumber("CL_raiseL4", m_raiseL4);
    // SmartDashboard.putNumber("CL_gatehookRestHeight", m_gatehookRestHeight);

    // Field for manually progamming climber height
    //MAKE THESE READ DEGREES OF THE MOTORS (BOTH SEPERATE)
    SmartDashboard.putNumber("EL_curDegrees", m_curELDegrees);
    SmartDashboard.putNumber("EL_targetDegrees", m_targetELDegrees);
    SmartDashboard.putBoolean("EL_calibrated", m_calibrated);

    SmartDashboard.putNumber("WR_curDegrees", m_curWRDegrees);
    SmartDashboard.putNumber("WR_targetDegrees", m_targetWRDegrees);
    SmartDashboard.putBoolean("WR_calibrated", m_calibrated);

    if (m_validEL14)
      climberTalonInitialize(m_Arm14, true);
    if (m_validWR15)
      climberTalonInitialize(m_Arm15, false);

    initialize( );
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
    // if disabled, set LED when down

    if (m_validEL14)
    {
      int curCounts = (int) m_Arm14.getSelectedSensorPosition(0);
      m_curELDegrees = countsToDegrees(curCounts);
      SmartDashboard.putNumber("ARM_curDegrees", m_curELDegrees);
    }
    if (m_validWR15)
    {
      int curCounts = (int) m_Arm15.getSelectedSensorPosition(0);
      m_curWRDegrees = countsToDegrees(curCounts);
      SmartDashboard.putNumber("ARM_curDegrees", m_curWRDegrees);
    }
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
  }

  public void initialize( )
  {
    double curELCounts = 0.0;
    double curWRCounts = 0.0;

    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");

    setARMStopped( );

    if (m_validEL14)
      curELCounts = m_Arm14.getSelectedSensorPosition(0);
    m_curELDegrees = countsToDegrees((int) curELCounts);
    m_targetELDegrees = m_curELDegrees;
    DataLogManager.log(getSubsystem( ) + ": Init Target Inches: " + m_targetELDegrees);

    if (m_validWR15)
      curWRCounts = m_Arm15.getSelectedSensorPosition(0);
    m_curWRDegrees = countsToDegrees((int) curWRCounts);
    m_targetWRDegrees = m_curWRDegrees;
    DataLogManager.log(getSubsystem( ) + ": Init Target Inches: " + m_targetWRDegrees);

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
