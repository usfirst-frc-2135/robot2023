//
// Arm subystem - wrist joint
//
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Falcon500;
import frc.robot.Constants.WRConsts;
import frc.robot.Constants.WRConsts.WristAngle;
import frc.robot.Constants.WRConsts.WristMode;
import frc.robot.team2135.PhoenixUtil;

//
// Wrist subsystem class
//
public class Wrist extends SubsystemBase
{
  // Constants
  private static final int                CANTIMEOUT            = 30;  // CAN timeout in msec
  private static final int                PIDINDEX              = 0;   // PID in use (0-primary, 1-aux)
  private static final int                SLOTINDEX             = 0;   // Use first PID slot

  // Member objects
  private final WPI_TalonFX               m_wrist               = new WPI_TalonFX(Constants.Ports.kCANID_Wrist);  //wrist
  private final TalonFXSimCollection      m_wrist16MotorSim     = new TalonFXSimCollection(m_wrist);
  private final SingleJointedArmSim       m_wrist16Sim          = new SingleJointedArmSim(DCMotor.getFalcon500(1),
      WRConsts.kWristGearRatio, 2.0, WRConsts.kGripperLengthMeters, 0.0, Math.PI, true);

  private final MechanismLigament2d       m_wristLigament;

  private boolean                         m_wristValid;               // Health indicator for climber Talon 

  //Devices and simulation objs
  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true,
      Falcon500.kSupplyCurrentLimit, Falcon500.kSupplyTriggerCurrent, Falcon500.kSupplyTriggerTime);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true,
      Falcon500.kStatorCurrentLimit, Falcon500.kStatorTriggerCurrent, Falcon500.kStatorTriggerTime);

  // Declare module variables
  private int                             m_velocity            = WRConsts.kWristMMVelocity;         // motion magic velocity
  private int                             m_acceleration        = WRConsts.kWristMMAcceleration;     // motion magic acceleration
  private int                             m_sCurveStrength      = WRConsts.kWristMMSCurveStrength;   // motion magic S curve smoothing
  private double                          m_pidKf               = WRConsts.kWristPidKf;           // PID force constant
  private double                          m_pidKp               = WRConsts.kWristPidKp;           // PID proportional
  private double                          m_pidKi               = WRConsts.kWristPidKi;           // PID integral
  private double                          m_pidKd               = WRConsts.kWristPidKd;           // PID derivative
  private int                             m_wristAllowedError   = WRConsts.kWristAllowedError;     // PID allowable closed loop error
  private double                          m_toleranceInches     = WRConsts.kWristToleranceDegrees; // PID tolerance in inches

  private double                          m_wristStowangle      = WRConsts.kWristStowAngle;    // wrist Stow angle
  private double                          m_lowScoreangle       = WRConsts.kWristAngleScoreLow;     // low-peg scoring angle   
  private double                          m_midScoreangle       = WRConsts.kWristAngleScoreMid;     // mid-peg scoring angle
  private double                          m_highScoreangle      = WRConsts.kWristAngleScoreHigh;    // high-peg scoring angle
  private double                          m_wristMinangle       = WRConsts.kWristMinAngle;       // minimum wrist allowable angle
  private double                          m_wristMaxangle       = WRConsts.kWristMaxAngle;       // maximum wrist allowable angle

  private double                          m_stickDeadband       = Constants.kStickDeadband;      // joystick deadband
  private WristMode                       m_wristMode           = WristMode.WRIST_INIT;          // Mode active with joysticks

  private int                             m_wristDebug          = 1; // DEBUG flag to disable/enable extra logging calls

  private boolean                         m_calibrated          = false;  // Indicates whether the climber has been calibrated
  private double                          m_wristTargetDegrees  = 0.0;    // Target angle in degrees
  private double                          m_wristCurDegrees     = 0.0;    // Current angle in degrees
  private int                             m_withinTolerance     = 0;      // Counter for consecutive readings within tolerance

  private Timer                           m_safetyTimer         = new Timer( ); // Safety timer for use in climber
  private double                          m_safetyTimeout;                // Seconds that the timer ran before stopping

  // Constructor
  public Wrist( )
  {
    setName("Wrist");
    setSubsystem("Wrist");

    m_wristValid = PhoenixUtil.getInstance( ).talonFXInitialize(m_wrist, "wrist");

    // SmartDashboard.putBoolean("HL_validWR", m_validWR);

    // // Initialize Variables
    // SmartDashboard.putNumber("WR_velocity", m_velocity);
    // SmartDashboard.putNumber("WR_acceleration", m_acceleration);
    // SmartDashboard.putNumber("WR_sCurveStrength", m_sCurveStrength);
    // SmartDashboard.putNumber("WR_pidKf", m_pidKf);
    // SmartDashboard.putNumber("WR_pidKp", m_pidKp);
    // SmartDashboard.putNumber("WR_pidKi", m_pidKi);
    // SmartDashboard.putNumber("WR_pidKd", m_pidKd);

    // SmartDashboard.putNumber("WR_stowangle", m_stowangle);

    SmartDashboard.putNumber("WR_curDegrees", m_wristCurDegrees);
    SmartDashboard.putNumber("WR_targetDegrees", m_wristTargetDegrees);
    SmartDashboard.putBoolean("WR_calibrated", m_calibrated);

    if (m_wristValid)
      wristTalonInitialize(m_wrist, false);

    // the main mechanism object
    Mechanism2d wristMech = new Mechanism2d(3, 3);
    // the mechanism root node
    MechanismRoot2d wristRoot = wristMech.getRoot("wrist", 1.5, 2);

    // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
    // off the root node or another ligament object
    m_wristLigament = wristRoot.append(new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));

    // post the mechanism to the dashboard
    SmartDashboard.putData("WristMech2d", wristMech);

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

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

    // Set input motor voltage from the motor setting
    m_wrist16MotorSim.setBusVoltage(RobotController.getInputVoltage( ));
    m_wrist16Sim.setInput(-m_wrist16MotorSim.getMotorOutputLeadVoltage( ));

    // update for 20 msec loop
    m_wrist16Sim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_wrist.setSelectedSensorPosition(wristDegreesToCounts(Units.radiansToDegrees(m_wrist16Sim.getAngleRads( ))));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_wrist16Sim.getCurrentDrawAmps( )));
  }

  public void initialize( )
  {
    double curWRCounts = 0.0;

    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");

    setWristStopped( );

    if (m_wristValid)
      curWRCounts = m_wrist.getSelectedSensorPosition(0);
    m_wristCurDegrees = wristCountsToDegrees((int) curWRCounts);
    m_wristTargetDegrees = m_wristCurDegrees;
    DataLogManager.log(getSubsystem( ) + ": Init Target Inches: " + m_wristTargetDegrees);
  }

  private int wristDegreesToCounts(double degrees)
  {
    return (int) (degrees / WRConsts.kWristDegreesPerCount);
  }

  private double wristCountsToDegrees(int counts)
  {
    return counts * WRConsts.kWristDegreesPerCount;
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

  public void moveWristWithJoystick(XboxController joystick)
  {
    double yWristValue = 0.0;
    double motorOutput = 0.0;
    double manualSpeedMax = WRConsts.kWristSpeedMaxManual;

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

  public void setWristStopped( )
  {
    DataLogManager.log(getSubsystem( ) + ": ARM Set Arm Stopped");

    if (m_wristValid)
      m_wrist.set(ControlMode.PercentOutput, 0.0);
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveWristDistanceInit(WristAngle angle)
  {
    if (m_wristDebug != 0)
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

    switch (angle)
    {
      case WRIST_NOCHANGE : // Do not change from current level!
        m_wristTargetDegrees = m_wristCurDegrees;
        if (m_wristTargetDegrees < 0.25)
          m_wristTargetDegrees = 0.25;
        break;
      case WRIST_STOW :
        m_wristTargetDegrees = SmartDashboard.getNumber("WR_stowangle", m_wristStowangle);
        break;
      case WRIST_LOW :
        m_wristTargetDegrees = SmartDashboard.getNumber("WR_lowScoreangle", m_lowScoreangle);
        break;
      case WRIST_MID :
        m_wristTargetDegrees = SmartDashboard.getNumber("WR_midScoreangle", m_midScoreangle);
        break;
      case WRIST_HIGH :
        m_wristTargetDegrees = SmartDashboard.getNumber("WR_highScoreangle", m_highScoreangle);
        break;
      default :
        DataLogManager.log(getSubsystem( ) + ": requested angle is invalid - " + angle);
        return;
    }

    if (m_calibrated)
    {
      // angle constraint check/soft limit for max and min angle before raising
      if (m_wristTargetDegrees < m_wristMinangle)
      {
        DataLogManager.log("Target " + String.format("%.1f", m_wristTargetDegrees) + " degrees is limited by "
            + String.format("%.1f", m_wristMinangle) + " degrees");
        m_wristTargetDegrees = m_wristMinangle;
      }

      if (m_wristTargetDegrees > m_wristMaxangle)
      {
        DataLogManager.log("Target " + String.format("%.1f", m_wristTargetDegrees) + " degrees is limited by "
            + String.format("%.1f", m_wristMaxangle) + " degrees");
        m_wristTargetDegrees = m_wristMaxangle;
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
}
