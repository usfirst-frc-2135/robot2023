// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ARMConsts;
import frc.robot.Constants.ARMConsts.ElbowMode;
import frc.robot.Constants.ARMConsts.WristMode;
import frc.robot.Constants.Falcon500;
import frc.robot.team2135.PhoenixUtil;

public class Arm extends SubsystemBase
{
  // Constants
  private static final int                CANTIMEOUT            = 30;  // CAN timeout in msec
  private static final int                PIDINDEX              = 0;   // PID in use (0-primary, 1-aux)
  private static final int                SLOTINDEX             = 0;   // Use first PID slot

  private final WPI_TalonFX               m_elbow               = new WPI_TalonFX(Constants.Ports.kCANID_Elbow);  //elbow
  private final WPI_TalonFX               m_wrist               = new WPI_TalonFX(Constants.Ports.kCANID_Wrist);  //wrist
  private final TalonFXSimCollection      m_elbowMotorSim       = new TalonFXSimCollection(m_elbow);
  private final TalonFXSimCollection      m_wrist16MotorSim     = new TalonFXSimCollection(m_wrist);
  private final SingleJointedArmSim       m_elbowSim            = new SingleJointedArmSim(DCMotor.getFalcon500(1),
      ARMConsts.kElbowGearRatio, 2.0, ARMConsts.kForearmLengthMeters, 0.0, Math.PI, ARMConsts.kForearmMassKg, true);
  private final SingleJointedArmSim       m_wrist16Sim          = new SingleJointedArmSim(DCMotor.getFalcon500(1),
      ARMConsts.kWristGearRatio, 2.0, ARMConsts.kGripperLengthMeters, 0.0, Math.PI, ARMConsts.kGripperMassKg, true);

  private final MechanismLigament2d       m_elbowLigament;
  private final MechanismLigament2d       m_wristLigament;

  private boolean                         m_elbowValid;               // Health indicator for climber Talon 
  private boolean                         m_wristValid;               // Health indicator for climber Talon 

  //Devices and simulation objs
  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true,
      Falcon500.kSupplyCurrentLimit, Falcon500.kSupplyTriggerCurrent, Falcon500.kSupplyTriggerTime);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true,
      Falcon500.kStatorCurrentLimit, Falcon500.kStatorTriggerCurrent, Falcon500.kStatorTriggerTime);

  // Declare module variables
  private int                             m_velocity            = ARMConsts.kMMVelocity;        // motion magic velocity
  private int                             m_acceleration        = ARMConsts.kMMAcceleration;    // motion magic acceleration
  private int                             m_sCurveStrength      = ARMConsts.kMMSCurveStrength;  // motion magic S curve smoothing
  private double                          m_pidKf               = ARMConsts.kARMPidKf;           // PID force constant
  private double                          m_pidKp               = ARMConsts.kARMPidKp;           // PID proportional
  private double                          m_pidKi               = ARMConsts.kARMPidKi;           // PID integral
  private double                          m_pidKd               = ARMConsts.kARMPidKd;           // PID derivative
  private int                             m_elbowAllowedError   = ARMConsts.kELAllowedError;    // PID allowable closed loop error
  private int                             m_wristAllowedError   = ARMConsts.kWRAllowedError;    // PID allowable closed loop error
  private double                          m_toleranceInches     = ARMConsts.kARMToleranceInches; // PID tolerance in inches

  private double                          m_stowHeight          = ARMConsts.kStowHeight;         // Stow height

  private double                          m_stickDeadband       = ARMConsts.kStickDeadband;      // joystick deadband
  private ElbowMode                       m_elbowMode           = ElbowMode.ELBOW_INIT;          // Mode active with joysticks
  private WristMode                       m_wristMode           = WristMode.WRIST_INIT;          // Mode active with joysticks

  private int                             m_climberDebug        = 1; // DEBUG flag to disable/enable extra logging calls

  private boolean                         m_calibrated          = false;  // Indicates whether the climber has been calibrated
  private double                          m_elbowTargetDegrees  = 0.0;    // Target angle in degrees
  private double                          m_elbowCurDegrees     = 0.0;    // Current angle in degrees
  private double                          m_wristTargetDegrees  = 0.0;    // Target angle in degrees
  private double                          m_wristCurDegrees     = 0.0;    // Current angle in degrees
  private int                             m_withinTolerance     = 0;      // Counter for consecutive readings within tolerance

  private Timer                           m_safetyTimer         = new Timer( ); // Safety timer for use in climber
  private double                          m_safetyTimeout;                // Seconds that the timer ran before stopping
  private Timer                           timer                 = new Timer( );

  /**
   *
   */
  /** Creates a new ExampleSubsystem. */
  public Arm( )
  {
    setName("Arm");
    setSubsystem("Arm");

    m_elbowValid = PhoenixUtil.getInstance( ).talonFXInitialize(m_elbow, "elbow");
    m_wristValid = PhoenixUtil.getInstance( ).talonFXInitialize(m_wrist, "wrist");

    // SmartDashboard.putBoolean("HL_validCL", m_validCL);
    // SmartDashboard.putBoolean("HL_validCL", m_validCL);

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
    SmartDashboard.putNumber("EL_curDegrees", m_elbowCurDegrees);
    SmartDashboard.putNumber("EL_targetDegrees", m_elbowTargetDegrees);
    SmartDashboard.putBoolean("EL_calibrated", m_calibrated);

    SmartDashboard.putNumber("WR_curDegrees", m_wristCurDegrees);
    SmartDashboard.putNumber("WR_targetDegrees", m_wristTargetDegrees);
    SmartDashboard.putBoolean("WR_calibrated", m_calibrated);

    if (m_elbowValid)
      elbowTalonInitialize(m_elbow, false);
    if (m_wristValid)
      wristTalonInitialize(m_wrist, false);

    // the main mechanism object
    Mechanism2d armMech = new Mechanism2d(3, 3);
    // the mechanism root node
    MechanismRoot2d armRoot = armMech.getRoot("arm", 1.5, 2);

    // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
    // off the root node or another ligament object
    m_elbowLigament = armRoot.append(new MechanismLigament2d("elbow", 1, 0));
    m_wristLigament = m_elbowLigament.append(new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));

    // post the mechanism to the dashboard
    SmartDashboard.putData("ArmMech2d", armMech);

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    // if disabled, set LED when down

    if (m_elbowValid)
    {
      int curCounts = (int) m_elbow.getSelectedSensorPosition(0);
      m_elbowCurDegrees = elbowCountsToDegrees(curCounts);
      SmartDashboard.putNumber("EL_curDegrees", m_elbowCurDegrees);
      m_elbowLigament.setAngle(elbowCountsToDegrees(curCounts));
    }
    if (m_wristValid)
    {
      int curCounts = (int) m_wrist.getSelectedSensorPosition(0);
      m_wristCurDegrees = wristCountsToDegrees(curCounts);
      SmartDashboard.putNumber("WR_curDegrees", m_wristCurDegrees);
      m_wristLigament.setAngle(wristCountsToDegrees(curCounts));
    }
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input flywheel voltage from the motor setting
    m_elbowMotorSim.setBusVoltage(RobotController.getInputVoltage( ));
    m_elbowSim.setInput(-m_elbowMotorSim.getMotorOutputLeadVoltage( ));

    m_wrist16MotorSim.setBusVoltage(RobotController.getInputVoltage( ));
    m_wrist16Sim.setInput(-m_wrist16MotorSim.getMotorOutputLeadVoltage( ));

    // update for 20 msec loop
    m_elbowSim.update(0.020);
    m_wrist16Sim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_elbow.setSelectedSensorPosition(elbowDegreesToCounts(Units.radiansToDegrees(-m_elbowSim.getAngleRads( ))));
    m_wrist.setSelectedSensorPosition(wristDegreesToCounts(Units.radiansToDegrees(m_wrist16Sim.getAngleRads( ))));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elbowSim.getCurrentDrawAmps( )));
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_wrist16Sim.getCurrentDrawAmps( )));
  }

  public void initialize( )
  {
    double curELCounts = 0.0;
    double curWRCounts = 0.0;

    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");

    setARMStopped( );

    if (m_elbowValid)
      curELCounts = m_elbow.getSelectedSensorPosition(0);
    m_elbowCurDegrees = elbowCountsToDegrees((int) curELCounts);
    m_elbowTargetDegrees = m_elbowCurDegrees;
    DataLogManager.log(getSubsystem( ) + ": Init Target Inches: " + m_elbowTargetDegrees);

    if (m_wristValid)
      curWRCounts = m_wrist.getSelectedSensorPosition(0);
    m_wristCurDegrees = wristCountsToDegrees((int) curWRCounts);
    m_wristTargetDegrees = m_wristCurDegrees;
    DataLogManager.log(getSubsystem( ) + ": Init Target Inches: " + m_wristTargetDegrees);

  }

  private int elbowDegreesToCounts(double degrees)
  {
    return (int) (degrees / ARMConsts.kElbowDegreesPerCount);
  }

  private double elbowCountsToDegrees(int counts)
  {
    return counts * ARMConsts.kElbowDegreesPerCount;
  }

  private int wristDegreesToCounts(double degrees)
  {
    return (int) (degrees / ARMConsts.kElbowDegreesPerCount);
  }

  private double wristCountsToDegrees(int counts)
  {
    return counts * ARMConsts.kElbowDegreesPerCount;
  }

  private void elbowTalonInitialize(WPI_TalonFX motor, boolean inverted)
  {
    motor.setInverted(inverted);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setInverted");
    motor.setNeutralMode(NeutralMode.Brake);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setNeutralMode");
    motor.setSafetyEnabled(false);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setSafetyEnabled");

    motor.configVoltageCompSaturation(12.0);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configVoltageCompSaturation");
    motor.enableVoltageCompensation(true);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "enableVoltageCompensation");

    motor.configSupplyCurrentLimit(m_supplyCurrentLimits);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configSupplyCurrentLimits");
    motor.configStatorCurrentLimit(m_statorCurrentLimits);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configStatorCurrentLimits");

    // Configure sensor settings
    motor.setSelectedSensorPosition(0.0);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setSelectedSensorPosition");
    motor.configAllowableClosedloopError(SLOTINDEX, m_elbowAllowedError, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configAllowableClosedloopError");

    motor.configMotionCruiseVelocity(m_velocity, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configMotionCruiseVelocity");
    motor.configMotionAcceleration(m_acceleration, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configMotionAcceleration");
    motor.configMotionSCurveStrength(m_sCurveStrength, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configMotionSCurveStrength");

    // Configure Magic Motion settings
    motor.config_kF(0, m_pidKf, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kF");
    motor.config_kP(0, m_pidKp, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kP");
    motor.config_kI(0, m_pidKi, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kI");
    motor.config_kD(0, m_pidKd, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kD");
    motor.selectProfileSlot(SLOTINDEX, PIDINDEX);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "selectProfileSlot");

    motor.set(ControlMode.PercentOutput, 0.0);
  }

  private void wristTalonInitialize(WPI_TalonFX motor, boolean inverted)
  {
    motor.setInverted(inverted);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setInverted");
    motor.setNeutralMode(NeutralMode.Brake);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setNeutralMode");
    motor.setSafetyEnabled(false);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setSafetyEnabled");

    motor.configVoltageCompSaturation(12.0);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configVoltageCompSaturation");
    motor.enableVoltageCompensation(true);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "enableVoltageCompensation");

    motor.configSupplyCurrentLimit(m_supplyCurrentLimits);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configSupplyCurrentLimits");
    motor.configStatorCurrentLimit(m_statorCurrentLimits);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configStatorCurrentLimits");

    // Configure sensor settings
    motor.setSelectedSensorPosition(0.0);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setSelectedSensorPosition");
    motor.configAllowableClosedloopError(SLOTINDEX, m_wristAllowedError, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configAllowableClosedloopError");

    motor.configMotionCruiseVelocity(m_velocity, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configMotionCruiseVelocity");
    motor.configMotionAcceleration(m_acceleration, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configMotionAcceleration");
    motor.configMotionSCurveStrength(m_sCurveStrength, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configMotionSCurveStrength");

    // Configure Magic Motion settings
    motor.config_kF(0, m_pidKf, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kF");
    motor.config_kP(0, m_pidKp, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kP");
    motor.config_kI(0, m_pidKi, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kI");
    motor.config_kD(0, m_pidKd, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kD");
    motor.selectProfileSlot(SLOTINDEX, PIDINDEX);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "selectProfileSlot");

    motor.set(ControlMode.PercentOutput, 0.0);
  }

  public void moveElbowWithJoystick(XboxController joystick)
  {
    double yELBOWValue = 0.0;
    double motorOutput = 0.0;
    double manualSpeedMax = ARMConsts.kSpeedMaxManual;

    yELBOWValue = -joystick.getLeftY( );
    if (yELBOWValue > -m_stickDeadband && yELBOWValue < m_stickDeadband)
    {
      if (m_elbowMode != ElbowMode.ELBOW_STOPPED)
        DataLogManager.log(getSubsystem( ) + " ELBOW Stopped");
      m_elbowMode = ElbowMode.ELBOW_STOPPED;
    }
    else
    {
      // If joystick is above a value, elbow will move up
      if (yELBOWValue > m_stickDeadband)
      {
        if (m_elbowMode != ElbowMode.ELBOW_UP)
          DataLogManager.log(getSubsystem( ) + " ELBOW Up");
        m_elbowMode = ElbowMode.ELBOW_UP;

        yELBOWValue -= m_stickDeadband;
        yELBOWValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yELBOWValue * Math.abs(yELBOWValue));
      }
      // If joystick is below a value, elbow will move down
      else if (yELBOWValue < -m_stickDeadband)
      {
        if (m_elbowMode != ElbowMode.ELBOW_DOWN)
          DataLogManager.log(getSubsystem( ) + " ELBOW Down");
        m_elbowMode = ElbowMode.ELBOW_DOWN;

        yELBOWValue += m_stickDeadband;
        yELBOWValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yELBOWValue * Math.abs(yELBOWValue));
      }
    }

    if (m_elbowValid)
      m_elbow.set(ControlMode.PercentOutput, motorOutput);
  }

  public void moveWristWithJoystick(XboxController joystick)
  {
    double yWRISTValue = 0.0;
    double motorOutput = 0.0;
    double manualSpeedMax = ARMConsts.kSpeedMaxManual;

    yWRISTValue = -joystick.getRightY( );
    if (yWRISTValue > -m_stickDeadband && yWRISTValue < m_stickDeadband)
    {
      if (m_wristMode != WristMode.WRIST_STOPPED)
        DataLogManager.log(getSubsystem( ) + " WRIST Stopped");
      m_wristMode = WristMode.WRIST_STOPPED;
    }
    else
    {
      // If joystick is above a value, wrist will move up
      if (yWRISTValue > m_stickDeadband)
      {
        if (m_wristMode != WristMode.WRIST_UP)
          DataLogManager.log(getSubsystem( ) + " WRIST Up");
        m_wristMode = WristMode.WRIST_UP;

        yWRISTValue -= m_stickDeadband;
        yWRISTValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yWRISTValue * Math.abs(yWRISTValue));
      }
      // If joystick is below a value, wrist will move down
      else if (yWRISTValue < -m_stickDeadband)
      {
        if (m_wristMode != WristMode.WRIST_DOWN)
          DataLogManager.log(getSubsystem( ) + " WRIST Down");
        m_wristMode = WristMode.WRIST_DOWN;

        yWRISTValue += m_stickDeadband;
        yWRISTValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yWRISTValue * Math.abs(yWRISTValue));
      }
    }

    if (m_wristValid)
      m_wrist.set(ControlMode.PercentOutput, motorOutput);
  }

  public void setARMStopped( )
  {
    DataLogManager.log(getSubsystem( ) + ": ARM Set Arm Stopped");

    if (m_elbowValid)
      m_elbow.set(ControlMode.PercentOutput, 0.0);

    if (m_wristValid)
      m_wrist.set(ControlMode.PercentOutput, 0.0);
  }
}
