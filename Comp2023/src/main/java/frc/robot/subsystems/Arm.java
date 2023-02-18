//
// Arm subystem - elbow and wrist joints
//
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ARMConsts;
import frc.robot.Constants.ARMConsts.ElbowAngle;
import frc.robot.Constants.ARMConsts.ElbowMode;
import frc.robot.Constants.ARMConsts.WristAngle;
import frc.robot.Constants.ARMConsts.WristMode;
import frc.robot.Constants.Falcon500;
import frc.robot.team2135.PhoenixUtil;

//
// Arm subsystem class
//
public class Arm extends SubsystemBase
{
  // Constants
  private static final int                CANTIMEOUT            = 30;  // CAN timeout in msec
  private static final int                PIDINDEX              = 0;   // PID in use (0-primary, 1-aux)
  private static final int                SLOTINDEX             = 0;   // Use first PID slot

  // Member objects
  private final WPI_TalonFX               m_elbow               = new WPI_TalonFX(Constants.Ports.kCANID_Elbow);  //elbow
  private final WPI_TalonFX               m_wrist               = new WPI_TalonFX(Constants.Ports.kCANID_Wrist);  //wrist
  private final TalonFXSimCollection      m_elbowMotorSim       = new TalonFXSimCollection(m_elbow);
  private final TalonFXSimCollection      m_wrist16MotorSim     = new TalonFXSimCollection(m_wrist);
  private final SingleJointedArmSim       m_elbowSim            = new SingleJointedArmSim(DCMotor.getFalcon500(1),
      ARMConsts.kElbowGearRatio, 2.0, ARMConsts.kForearmLengthMeters, 0.0, Math.PI, true);
  private final SingleJointedArmSim       m_wrist16Sim          = new SingleJointedArmSim(DCMotor.getFalcon500(1),
      ARMConsts.kWristGearRatio, 2.0, ARMConsts.kGripperLengthMeters, 0.0, Math.PI, true);

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
  private int                             m_velocity            = ARMConsts.kMMVelocity;         // motion magic velocity
  private int                             m_acceleration        = ARMConsts.kMMAcceleration;     // motion magic acceleration
  private int                             m_sCurveStrength      = ARMConsts.kMMSCurveStrength;   // motion magic S curve smoothing
  private double                          m_pidKf               = ARMConsts.kARMPidKf;           // PID force constant
  private double                          m_pidKp               = ARMConsts.kARMPidKp;           // PID proportional
  private double                          m_pidKi               = ARMConsts.kARMPidKi;           // PID integral
  private double                          m_pidKd               = ARMConsts.kARMPidKd;           // PID derivative
  private int                             m_elbowAllowedError   = ARMConsts.kELAllowedError;     // PID allowable closed loop error
  private int                             m_wristAllowedError   = ARMConsts.kWRAllowedError;     // PID allowable closed loop error
  private double                          m_toleranceInches     = ARMConsts.kARMToleranceInches; // PID tolerance in inches

  private double                          m_elbowStowAngle      = ARMConsts.kElbowStowAngle;    // elbow Stow Angle
  private double                          m_wristStowAngle      = ARMConsts.kWristStowAngle;    // wrist Stow Angle
  private double                          m_lowScoreAngle       = ARMConsts.kLowScoreAngle;     // low-peg scoring Angle   
  private double                          m_midScoreAngle       = ARMConsts.kMidScoreAngle;     // mid-peg scoring Angle
  private double                          m_highScoreAngle      = ARMConsts.kHighScoreAngle;    // high-peg scoring Angle
  private double                          m_elbowMinAngle       = ARMConsts.kElbowMinAngle;       // minimum elbow allowable Angle
  private double                          m_elbowMaxAngle       = ARMConsts.kElbowMaxAngle;       // maximum elbow allowable Angle
  private double                          m_wristMinAngle       = ARMConsts.kWristMinAngle;       // minimum wrist allowable Angle
  private double                          m_wristMaxAngle       = ARMConsts.kWristMaxAngle;       // maximum wrist allowable Angle

  private double                          m_stickDeadband       = ARMConsts.kStickDeadband;      // joystick deadband
  private ElbowMode                       m_elbowMode           = ElbowMode.ELBOW_INIT;          // Mode active with joysticks
  private WristMode                       m_wristMode           = WristMode.WRIST_INIT;          // Mode active with joysticks

  private int                             m_armDebug            = 1; // DEBUG flag to disable/enable extra logging calls

  private boolean                         m_calibrated          = false;  // Indicates whether the climber has been calibrated
  private double                          m_elbowTargetDegrees  = 0.0;    // Target angle in degrees
  private double                          m_elbowCurDegrees     = 0.0;    // Current angle in degrees
  private double                          m_wristTargetDegrees  = 0.0;    // Target angle in degrees
  private double                          m_wristCurDegrees     = 0.0;    // Current angle in degrees
  private int                             m_withinTolerance     = 0;      // Counter for consecutive readings within tolerance

  private Timer                           m_safetyTimer         = new Timer( ); // Safety timer for use in climber
  private double                          m_safetyTimeout;                // Seconds that the timer ran before stopping
  private Timer                           timer                 = new Timer( );

  // Constructor
  public Arm( )
  {
    setName("Arm");
    setSubsystem("Arm");

    m_elbowValid = PhoenixUtil.getInstance( ).talonFXInitialize(m_elbow, "elbow");
    m_wristValid = PhoenixUtil.getInstance( ).talonFXInitialize(m_wrist, "wrist");

    // SmartDashboard.putBoolean("HL_validEL", m_validEL);
    // SmartDashboard.putBoolean("HL_validWR", m_validWR);

    // // Initialize Variables
    // SmartDashboard.putNumber("EL_velocity", m_velocity);
    // SmartDashboard.putNumber("EL_acceleration", m_acceleration);
    // SmartDashboard.putNumber("EL_sCurveStrength", m_sCurveStrength);
    // SmartDashboard.putNumber("EL_pidKf", m_pidKf);
    // SmartDashboard.putNumber("EL_pidKp", m_pidKp);
    // SmartDashboard.putNumber("EL_pidKi", m_pidKi);
    // SmartDashboard.putNumber("EL_pidKd", m_pidKd);

    // SmartDashboard.putNumber("EL_stowAngle", m_stowAngle);

    // Field for manually progamming elbow angle
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
    double yElbowValue = 0.0;
    double motorOutput = 0.0;
    double manualSpeedMax = ARMConsts.kSpeedMaxManual;

    yElbowValue = joystick.getLeftY( );
    if (yElbowValue > -m_stickDeadband && yElbowValue < m_stickDeadband)
    {
      if (m_elbowMode != ElbowMode.ELBOW_STOPPED)
        DataLogManager.log(getSubsystem( ) + " ELBOW Stopped");
      m_elbowMode = ElbowMode.ELBOW_STOPPED;
    }
    else
    {
      // If joystick is above a value, elbow will move up
      if (yElbowValue > m_stickDeadband)
      {
        if (m_elbowMode != ElbowMode.ELBOW_UP)
          DataLogManager.log(getSubsystem( ) + " ELBOW Up");
        m_elbowMode = ElbowMode.ELBOW_UP;

        yElbowValue -= m_stickDeadband;
        yElbowValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yElbowValue * Math.abs(yElbowValue));
      }
      // If joystick is below a value, elbow will move down
      else if (yElbowValue < -m_stickDeadband)
      {
        if (m_elbowMode != ElbowMode.ELBOW_DOWN)
          DataLogManager.log(getSubsystem( ) + " ELBOW Down");
        m_elbowMode = ElbowMode.ELBOW_DOWN;

        yElbowValue += m_stickDeadband;
        yElbowValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yElbowValue * Math.abs(yElbowValue));
      }
    }

    if (m_elbowValid)
      m_elbow.set(ControlMode.PercentOutput, motorOutput);
  }

  public void moveWristWithJoystick(XboxController joystick)
  {
    double yWristValue = 0.0;
    double motorOutput = 0.0;
    double manualSpeedMax = ARMConsts.kSpeedMaxManual;

    yWristValue = joystick.getRightY( );
    if (yWristValue > -m_stickDeadband && yWristValue < m_stickDeadband)
    {
      if (m_wristMode != WristMode.WRIST_STOPPED)
        DataLogManager.log(getSubsystem( ) + " WRIST Stopped");
      m_wristMode = WristMode.WRIST_STOPPED;
    }
    else
    {
      // If joystick is above a value, wrist will move up
      if (yWristValue > m_stickDeadband)
      {
        if (m_wristMode != WristMode.WRIST_UP)
          DataLogManager.log(getSubsystem( ) + " WRIST Up");
        m_wristMode = WristMode.WRIST_UP;

        yWristValue -= m_stickDeadband;
        yWristValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yWristValue * Math.abs(yWristValue));
      }
      // If joystick is below a value, wrist will move down
      else if (yWristValue < -m_stickDeadband)
      {
        if (m_wristMode != WristMode.WRIST_DOWN)
          DataLogManager.log(getSubsystem( ) + " WRIST Down");
        m_wristMode = WristMode.WRIST_DOWN;

        yWristValue += m_stickDeadband;
        yWristValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yWristValue * Math.abs(yWristValue));
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

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveElbowDistanceInit(ElbowAngle Angle)
  {
    if (m_armDebug != 0)
    {
      m_velocity = (int) SmartDashboard.getNumber("EL_velocity", m_velocity);
      m_acceleration = (int) SmartDashboard.getNumber("EL_acceleration", m_acceleration);
      m_sCurveStrength = (int) SmartDashboard.getNumber("EL_sCurveStrength", m_sCurveStrength);
      m_pidKf = SmartDashboard.getNumber("EL_pidKf", m_pidKf);
      m_pidKp = SmartDashboard.getNumber("EL_pidKp", m_pidKp);
      m_pidKi = SmartDashboard.getNumber("EL_pidKi", m_pidKi);
      m_pidKd = SmartDashboard.getNumber("EL_pidKd", m_pidKd);

      m_elbow.configMotionCruiseVelocity(m_velocity);
      m_elbow.configMotionAcceleration(m_acceleration);
      m_elbow.configMotionSCurveStrength(m_sCurveStrength);
      m_elbow.config_kF(SLOTINDEX, m_pidKf);
      m_elbow.config_kP(SLOTINDEX, m_pidKp);
      m_elbow.config_kI(SLOTINDEX, m_pidKi);
      m_elbow.config_kD(SLOTINDEX, m_pidKd);
    }

    switch (Angle)
    {
      case ELBOW_NOCHANGE : // Do not change from current level!
        m_elbowTargetDegrees = m_elbowCurDegrees;
        if (m_elbowTargetDegrees < 0.25)
          m_elbowTargetDegrees = 0.25;
        break;
      case ELBOW_STOW :
        m_elbowTargetDegrees = SmartDashboard.getNumber("EL_stowAngle", m_elbowStowAngle);
        break;
      case ELBOW_LOW :
        m_elbowTargetDegrees = SmartDashboard.getNumber("EL_lowScoreAngle", m_lowScoreAngle);
        break;
      case ELBOW_MID :
        m_elbowTargetDegrees = SmartDashboard.getNumber("EL_midScoreAngle", m_midScoreAngle);
        break;
      case ELBOW_HIGH :
        m_elbowTargetDegrees = SmartDashboard.getNumber("EL_highScoreAngle", m_highScoreAngle);
        break;
      default :
        DataLogManager.log(getSubsystem( ) + ": requested Angle is invalid - " + Angle);
        return;
    }

    if (m_calibrated)
    {
      // Angle constraint check/soft limit for max and min Angle before raising
      if (m_elbowTargetDegrees < m_elbowMinAngle)
      {
        DataLogManager.log("Target " + String.format("%.1f", m_elbowTargetDegrees) + " degrees is limited by "
            + String.format("%.1f", m_elbowMinAngle) + " degrees");
        m_elbowTargetDegrees = m_elbowMinAngle;
      }

      if (m_elbowTargetDegrees > m_elbowMaxAngle)
      {
        DataLogManager.log("Target " + String.format("%.1f", m_elbowTargetDegrees) + " degrees is limited by "
            + String.format("%.1f", m_elbowMaxAngle) + " degrees");
        m_elbowTargetDegrees = m_elbowMaxAngle;
      }

      // Start the safety timer
      m_safetyTimeout = 1.8;
      m_safetyTimer.reset( );
      m_safetyTimer.start( );

      m_elbow.set(ControlMode.MotionMagic, elbowDegreesToCounts(m_elbowTargetDegrees));

      DataLogManager.log("elbow moving: " + String.format("%.1f", m_elbowCurDegrees) + " -> "
          + String.format("%.1f", m_elbowTargetDegrees) + " degrees  |  counts " + elbowDegreesToCounts(m_elbowCurDegrees)
          + " -> " + elbowDegreesToCounts(m_elbowTargetDegrees));
    }
    else
    {
      DataLogManager.log("elbow is not calibrated");
      if (m_elbowValid)
        m_elbow.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public boolean moveElbowDistanceIsFinished( )
  {
    boolean isFinished = false;
    double errorInInches = 0.0;

    errorInInches = m_elbowTargetDegrees - m_elbowCurDegrees;

    if (Math.abs(errorInInches) < m_toleranceInches)
    {
      if (++m_withinTolerance >= 5)
      {
        isFinished = true;
        DataLogManager.log("elbow move finished - Time: " + String.format("%.3f", m_safetyTimer.get( )) + "  |  Cur degrees: "
            + String.format("%.1f", m_elbowCurDegrees));
      }
    }
    else
    {
      m_withinTolerance = 0;
    }

    if (m_safetyTimer.get( ) >= m_safetyTimeout)
    {
      isFinished = true;
      DataLogManager.log("Arm Move Safety timer has timed out");
    }

    if (isFinished)
    {
      m_withinTolerance = 0;
      m_safetyTimer.stop( );
    }

    return isFinished;
  }

  public void moveWristDistanceInit(WristAngle Angle)
  {
    if (m_armDebug != 0)
    {
      m_velocity = (int) SmartDashboard.getNumber("WR_velocity", m_velocity);
      m_acceleration = (int) SmartDashboard.getNumber("WR_acceleration", m_acceleration);
      m_sCurveStrength = (int) SmartDashboard.getNumber("WR_sCurveStrength", m_sCurveStrength);
      m_pidKf = SmartDashboard.getNumber("WR_pidKf", m_pidKf);
      m_pidKp = SmartDashboard.getNumber("WR_pidKp", m_pidKp);
      m_pidKi = SmartDashboard.getNumber("WR_pidKi", m_pidKi);
      m_pidKd = SmartDashboard.getNumber("WR_pidKd", m_pidKd);

      m_wrist.configMotionCruiseVelocity(m_velocity);
      m_wrist.configMotionAcceleration(m_acceleration);
      m_wrist.configMotionSCurveStrength(m_sCurveStrength);
      m_wrist.config_kF(SLOTINDEX, m_pidKf);
      m_wrist.config_kP(SLOTINDEX, m_pidKp);
      m_wrist.config_kI(SLOTINDEX, m_pidKi);
      m_wrist.config_kD(SLOTINDEX, m_pidKd);
    }

    switch (Angle)
    {
      case WRIST_NOCHANGE : // Do not change from current level!
        m_wristTargetDegrees = m_elbowCurDegrees;
        if (m_wristTargetDegrees < 0.25)
          m_wristTargetDegrees = 0.25;
        break;
      case WRIST_STOW :
        m_wristTargetDegrees = SmartDashboard.getNumber("WR_stowAngle", m_wristStowAngle);
        break;
      default :
        DataLogManager.log(getSubsystem( ) + ": requested Angle is invalid - " + Angle);
        return;
    }

    if (m_calibrated)
    {
      // Angle constraint check/soft limit for max and min Angle before raising
      if (m_wristTargetDegrees < m_wristMinAngle)
      {
        DataLogManager.log("Target " + String.format("%.1f", m_wristTargetDegrees) + " degrees is limited by "
            + String.format("%.1f", m_wristMinAngle) + " degrees");
        m_wristTargetDegrees = m_wristMinAngle;
      }

      if (m_wristTargetDegrees > m_wristMaxAngle)
      {
        DataLogManager.log("Target " + String.format("%.1f", m_wristTargetDegrees) + " degrees is limited by "
            + String.format("%.1f", m_wristMaxAngle) + " degrees");
        m_wristTargetDegrees = m_wristMaxAngle;
      }

      // Start the safety timer
      m_safetyTimeout = 1.8;
      m_safetyTimer.reset( );
      m_safetyTimer.start( );

      m_wrist.set(ControlMode.MotionMagic, wristDegreesToCounts(m_wristTargetDegrees));

      DataLogManager.log("wrist moving: " + String.format("%.1f", m_wristCurDegrees) + " -> "
          + String.format("%.1f", m_wristTargetDegrees) + " degrees  |  counts " + wristDegreesToCounts(m_wristCurDegrees)
          + " -> " + wristDegreesToCounts(m_wristTargetDegrees));
    }
    else
    {
      DataLogManager.log("wrist is not calibrated");
      if (m_wristValid)
        m_wrist.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public boolean moveWristDistanceIsFinished( )
  {
    boolean isFinished = false;
    double errorInInches = 0.0;

    errorInInches = m_wristTargetDegrees - m_wristCurDegrees;

    if (Math.abs(errorInInches) < m_toleranceInches)
    {
      if (++m_withinTolerance >= 5)
      {
        isFinished = true;
        DataLogManager.log("Wrist move finished - Time: " + String.format("%.3f", m_safetyTimer.get( )) + "  |  Cur degrees: "
            + String.format("%.1f", m_wristCurDegrees));
      }
    }
    else
    {
      m_withinTolerance = 0;
    }

    if (m_safetyTimer.get( ) >= m_safetyTimeout)
    {
      isFinished = true;
      DataLogManager.log("Arm Move Safety timer has timed out");
    }

    if (isFinished)
    {
      m_withinTolerance = 0;
      m_safetyTimer.stop( );
    }

    return isFinished;
  }

  public void timerStart( )
  {
    timer.reset( );
    timer.start( );
  }

  public void timerPrint( )
  {
    timer.stop( );
    DataLogManager.log("Arm Time: " + String.format("%.3f", timer.get( )) + " seconds");
  }
}
