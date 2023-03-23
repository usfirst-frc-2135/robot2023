//
// Arm subystem - wrist joint
//
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.Constants.WRConsts;
import frc.robot.Constants.WRConsts.WristAngle;
import frc.robot.Constants.WRConsts.WristMode;
import frc.robot.RobotContainer;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.CTREConfigs;
import frc.robot.team2135.PhoenixUtil;

//
// Wrist subsystem class
//
public class Wrist extends SubsystemBase
{
  // Constants
  private static final int                PIDINDEX              = 0;   // PID in use (0-primary, 1-aux)
  private static final int                SLOTINDEX             = 0;   // Use first PID slot

  // Member objects
  private final WPI_TalonFX               m_wrist               = new WPI_TalonFX(Constants.Ports.kCANID_Wrist);  //wrist
  private final CANCoder                  m_wristCANCoder       = new CANCoder(Constants.Ports.kCANID_WRCANCoder);
  private final TalonFXSimCollection      m_wristMotorSim       = new TalonFXSimCollection(m_wrist);
  private final SingleJointedArmSim       m_wristSim            = new SingleJointedArmSim(DCMotor.getFalcon500(1),
      WRConsts.kWristGearRatio, 2.0, WRConsts.kGripperLengthMeters, -Math.PI, Math.PI, false);

  // Mechanism2d
  private final Mechanism2d               m_wristMech           = new Mechanism2d(3, 3);
  private final MechanismRoot2d           m_wristRoot           = m_wristMech.getRoot("wrist", 1.5, 2);
  private final MechanismLigament2d       m_wristLigament       =
      m_wristRoot.append(new MechanismLigament2d("wrist", 0.5, 0, 6, new Color8Bit(Color.kPurple)));

  private boolean                         m_wristValid;                 // Health indicator for wrist Talon 
  private boolean                         m_wristCCValid;               // Health indicator for wrist CANCoder 

  //Devices and simulation objs
  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true,
      WRConsts.kSupplyCurrentLimit, WRConsts.kSupplyTriggerCurrent, WRConsts.kSupplyTriggerTime);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true,
      WRConsts.kStatorCurrentLimit, WRConsts.kStatorTriggerCurrent, WRConsts.kStatorTriggerTime);

  // Declare module variables
  private double                          m_neutralDeadband     = WRConsts.kWristNeutralDeadband;   // motor output deadband
  private int                             m_velocity            = WRConsts.kWristMMVelocity;         // motion magic velocity
  private int                             m_acceleration        = WRConsts.kWristMMAcceleration;     // motion magic acceleration
  private int                             m_sCurveStrength      = WRConsts.kWristMMSCurveStrength;   // motion magic S curve smoothing
  private double                          m_pidKf               = WRConsts.kWristPidKf;              // PID force constant
  private double                          m_pidKp               = WRConsts.kWristPidKp;              // PID proportional
  private double                          m_pidKi               = WRConsts.kWristPidKi;              // PID integral
  private double                          m_pidKd               = WRConsts.kWristPidKd;              // PID derivative
  private int                             m_wristAllowedError   = WRConsts.kWristAllowedError;       // PID allowable closed loop error
  private double                          m_toleranceDegrees    = WRConsts.kWristToleranceDegrees;   // PID tolerance in Degrees

  private double                          m_wristAngleMin       = WRConsts.kWristAngleMin;           // minimum wrist allowable angle
  private double                          m_wristAngleStow      = WRConsts.kWristAngleStow;          // wrist Stow angle
  private double                          m_wristAngleIdle      = WRConsts.kWristAngleIdle;          // wrist Idle angle  
  private double                          m_wristAngleLow       = WRConsts.kWristAngleScoreLow;      // low-peg scoring angle   
  private double                          m_wristAngleMid       = WRConsts.kWristAngleScoreMid;      // mid-peg scoring angle
  private double                          m_wristAngleHigh      = WRConsts.kWristAngleScoreHigh;     // high-peg scoring angle
  private double                          m_wristAngleScore     = WRConsts.kWristAngleScore;         // scoring angle
  private double                          m_wristAngleShelf     = WRConsts.kWristAngleSubstation;    // substation loading shelf
  private double                          m_wristAngleMax       = WRConsts.kWristAngleMax;           // maximum wrist allowable angle

  private double                          m_stickDeadband       = Constants.kStickDeadband;          // joystick deadband
  private WristMode                       m_wristMode           = WristMode.WRIST_INIT;              // Mode active with joysticks

  private boolean                         m_wristDebug          = false;  // DEBUG flag to disable/enable extra logging calls

  private WristAngle                      m_wristAngle;                   // Desired extension length
  private boolean                         m_moveIsFinished;
  private double                          m_wristTargetDegrees  = 0.0;    // Target angle in degrees
  private double                          m_wristCurDegrees     = 0.0;    // Current angle in degrees
  private int                             m_withinTolerance     = 0;      // Counter for consecutive readings within tolerance
  private double                          m_wristTotalFF;

  private Timer                           m_safetyTimer         = new Timer( ); // Safety timer for use in wrist

  private int                             maxVelocity;

  // Constructor
  public Wrist( )
  {
    setName("Wrist");
    setSubsystem("Wrist");

    m_wristValid = PhoenixUtil.getInstance( ).talonFXInitialize(m_wrist, "wrist");

    if (m_wristValid)
      wristTalonInitialize(m_wrist, WRConsts.kInvertMotor);

    m_wristCCValid = PhoenixUtil.getInstance( ).canCoderInitialize(m_wristCANCoder, "wrist");

    if (m_wristCCValid)
    {
      m_wristCANCoder.configAllSettings(CTREConfigs.wristCancoderConfig( ));
      double m_wristCurDegrees = getCanCoder( ).getDegrees( );

      // Slow status frame updates AFTER getting the absolute position
      m_wristCANCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
      m_wristCANCoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);

      DataLogManager.log(getSubsystem( ) + ": Initial degrees " + m_wristCurDegrees);
      double absolutePosition = Conversions.degreesToFalcon(m_wristCurDegrees, WRConsts.kWristGearRatio);

      if (RobotBase.isReal( ))
        m_wrist.setSelectedSensorPosition(absolutePosition);
    }

    m_wrist.configReverseSoftLimitThreshold(Conversions.degreesToFalcon(m_wristAngleMin, WRConsts.kWristGearRatio),
        Constants.kCANTimeoutMs);
    m_wrist.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);
    m_wrist.configForwardSoftLimitThreshold(Conversions.degreesToFalcon(m_wristAngleMax, WRConsts.kWristGearRatio),
        Constants.kCANTimeoutMs);
    m_wrist.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);

    initSmartDashboard( );

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    if (m_wristValid)
    {
      int curCounts = (int) m_wrist.getSelectedSensorPosition(0);

      if (m_wristDebug)
      {
        int curVelocity = (int) m_wrist.getSelectedSensorVelocity(0);
        maxVelocity = (maxVelocity > curVelocity) ? maxVelocity : curVelocity;
        SmartDashboard.putNumber("WR_maxVelocity", maxVelocity);
        SmartDashboard.putNumber("WR_curVelocity", curVelocity);
        SmartDashboard.putNumber("WR_curCounts", curCounts);
      }

      m_wristCurDegrees = wristCountsToDegrees(curCounts);
      SmartDashboard.putNumber("WR_curDegrees", m_wristCurDegrees);
      SmartDashboard.putNumber("WR_targetDegrees", m_wristTargetDegrees);
      m_wristLigament.setAngle(m_wristCurDegrees);

      m_wristTotalFF = calculateTotalFF( );
      SmartDashboard.putNumber("WR_totalFF", m_wristTotalFF);

      double currentDraw = m_wrist.getStatorCurrent( );
      SmartDashboard.putNumber("WR_currentDraw", currentDraw);
    }
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input motor voltage from the motor setting
    m_wristMotorSim.setBusVoltage(RobotController.getInputVoltage( ));
    m_wristSim.setInput(m_wristMotorSim.getMotorOutputLeadVoltage( ));

    // update for 20 msec loop
    m_wristSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_wristMotorSim.setIntegratedSensorRawPosition(wristDegreesToCounts(Units.radiansToDegrees(m_wristSim.getAngleRads( ))));
    m_wristMotorSim.setIntegratedSensorVelocity(wristDegreesToCounts(Units.radiansToDegrees(m_wristSim.getVelocityRadPerSec( ))));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_wristSim.getCurrentDrawAmps( )));
  }

  private void initSmartDashboard( )
  {
    SmartDashboard.putBoolean("HL_validWR", m_wristValid);

    // Initialize Variables
    SmartDashboard.putNumber("WR_velocity", m_velocity);
    SmartDashboard.putNumber("WR_acceleration", m_acceleration);
    SmartDashboard.putNumber("WR_sCurveStrength", m_sCurveStrength);
    SmartDashboard.putNumber("WR_pidKf", m_pidKf);
    SmartDashboard.putNumber("WR_pidKp", m_pidKp);
    SmartDashboard.putNumber("WR_pidKi", m_pidKi);
    SmartDashboard.putNumber("WR_pidKd", m_pidKd);

    SmartDashboard.putNumber("WR_angleStow", m_wristAngleStow);
    SmartDashboard.putNumber("WR_angleIdle", m_wristAngleIdle);
    SmartDashboard.putNumber("WR_angleLow", m_wristAngleLow);
    SmartDashboard.putNumber("WR_angleMid", m_wristAngleMid);
    SmartDashboard.putNumber("WR_angleHigh", m_wristAngleHigh);
    SmartDashboard.putNumber("WR_angleShelf", m_wristAngleShelf);

    SmartDashboard.putNumber("WR_curDegrees", m_wristCurDegrees);
    SmartDashboard.putNumber("WR_targetDegrees", m_wristTargetDegrees);
    SmartDashboard.putBoolean("WR_calibrated", WRConsts.kWristCalibrated);
    SmartDashboard.putBoolean("WR_normalMode", !m_wristDebug);

    // post the mechanism to the dashboard
    SmartDashboard.putData("WristMech", m_wristMech);
  }

  public void initialize( )
  {
    double curWRCounts = 0.0;

    DataLogManager.log(getSubsystem( ) + ": Subsystem initialized!");

    setWristStopped( );

    if (m_wristValid)
      curWRCounts = m_wrist.getSelectedSensorPosition(0);
    m_wristCurDegrees = wristCountsToDegrees((int) curWRCounts);
    m_wristTargetDegrees = m_wristCurDegrees;
    DataLogManager.log(String.format("%s: Init Target Degrees: %.1f", getSubsystem( ), m_wristTargetDegrees));
  }

  public Rotation2d getCanCoder( )
  {
    return Rotation2d.fromDegrees(m_wristCANCoder.getAbsolutePosition( ));
  }

  private int wristDegreesToCounts(double degrees)
  {
    return (int) (degrees / WRConsts.kWristDegreesPerCount);
  }

  private double wristCountsToDegrees(int counts)
  {
    return counts * WRConsts.kWristDegreesPerCount;
  }

  public boolean moveIsInRange(double degrees)
  {
    return (degrees > m_wristAngleMin) && (degrees < m_wristAngleMax);
  }

  public double getAngle( )
  {
    return m_wristCurDegrees;
  }

  private void wristTalonInitialize(WPI_TalonFX motor, boolean inverted)
  {
    motor.setInverted(inverted);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setInverted");
    motor.setNeutralMode(NeutralMode.Brake);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setNeutralMode");
    motor.setSafetyEnabled(false);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setSafetyEnabled");
    motor.configNeutralDeadband(m_neutralDeadband, Constants.kLongCANTimeoutMs);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configNeutralDeadband");

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
    motor.configAllowableClosedloopError(SLOTINDEX, m_wristAllowedError, Constants.kLongCANTimeoutMs);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configAllowableClosedloopError");

    motor.configMotionCruiseVelocity(m_velocity, Constants.kLongCANTimeoutMs);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configMotionCruiseVelocity");
    motor.configMotionAcceleration(m_acceleration, Constants.kLongCANTimeoutMs);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configMotionAcceleration");
    motor.configMotionSCurveStrength(m_sCurveStrength, Constants.kLongCANTimeoutMs);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configMotionSCurveStrength");

    // Configure Magic Motion settings
    motor.config_kF(0, m_pidKf, Constants.kLongCANTimeoutMs);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kF");
    motor.config_kP(0, m_pidKp, Constants.kLongCANTimeoutMs);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kP");
    motor.config_kI(0, m_pidKi, Constants.kLongCANTimeoutMs);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kI");
    motor.config_kD(0, m_pidKd, Constants.kLongCANTimeoutMs);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kD");
    motor.selectProfileSlot(SLOTINDEX, PIDINDEX);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "selectProfileSlot");

    motor.set(ControlMode.PercentOutput, 0.0);
  }

  public void moveWristWithJoystick(XboxController joystick)
  {
    double axisValue = -joystick.getRightY( );
    boolean outOfRange = false;
    WristMode newMode = WristMode.WRIST_STOPPED;

    axisValue = MathUtil.applyDeadband(axisValue, m_stickDeadband);

    if (axisValue < 0.0)
    {
      if (m_wristCurDegrees > m_wristAngleMin)
        newMode = WristMode.WRIST_UP;
      else
        outOfRange = true;
    }
    else if (axisValue > 0.0)
    {
      if (m_wristCurDegrees < m_wristAngleMax)
        newMode = WristMode.WRIST_DOWN;
      else
        outOfRange = true;
    }

    if (outOfRange)
      axisValue = 0.0;

    if (newMode != m_wristMode)
    {
      m_wristMode = newMode;
      DataLogManager.log(getSubsystem( ) + ": move " + m_wristMode + ((outOfRange) ? " - OUT OF RANGE" : ""));
    }

    m_wristTargetDegrees = m_wristCurDegrees;

    if (m_wristValid)
      m_wrist.set(ControlMode.PercentOutput, axisValue * WRConsts.kWristSpeedMaxManual + m_wristTotalFF);
  }

  public void setWristStopped( )
  {
    DataLogManager.log(getSubsystem( ) + ": now STOPPED");

    if (m_wristValid)
      m_wrist.set(ControlMode.PercentOutput, 0.0);
  }

  //m_wrsit::methodYouWantToDO
  public void setWristAngleToZero( )
  {
    m_wrist.setSelectedSensorPosition(wristDegreesToCounts(0));
  }

  private double calculateTotalFF( )
  {
    double elbowDegrees = RobotContainer.getInstance( ).m_elbow.getAngle( );
    double wristDegrees = RobotContainer.getInstance( ).m_wrist.getAngle( );

    return WRConsts.kWristArbitraryFF * Math.cos(Math.toRadians(elbowDegrees - wristDegrees));
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveWristAngleInit(WristAngle angle)
  {
    if (m_wristDebug)
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

      m_wristAngleStow = SmartDashboard.getNumber("WR_angleStow", m_wristAngleStow);
      m_wristAngleIdle = SmartDashboard.getNumber("WR_angleIdle", m_wristAngleIdle);
      m_wristAngleLow = SmartDashboard.getNumber("WR_angleLow", m_wristAngleLow);
      m_wristAngleMid = SmartDashboard.getNumber("WR_angleMid", m_wristAngleMid);
      m_wristAngleHigh = SmartDashboard.getNumber("WR_angleHigh", m_wristAngleHigh);
      m_wristAngleShelf = SmartDashboard.getNumber("WR_angleShelf", m_wristAngleShelf);
    }

    if (angle != m_wristAngle)
    {
      m_wristAngle = angle;
      m_moveIsFinished = false;
      DataLogManager.log(String.format("%s: new mode request - %s", getSubsystem( ), m_wristAngle));

      switch (m_wristAngle)
      {
        default : // Fall through to NOCHANGE if invalid
          DataLogManager.log(String.format("%s: requested angle is invalid - %s", getSubsystem( ), m_wristAngle));
        case WRIST_NOCHANGE : // Do not change from current level!
          m_wristTargetDegrees = m_wristCurDegrees;
          if (m_wristTargetDegrees < 0.25)
            m_wristTargetDegrees = 0.25;
          break;
        case WRIST_STOW :
          m_wristTargetDegrees = m_wristAngleStow;
          break;
        case WRIST_IDLE :
          m_wristTargetDegrees = m_wristAngleIdle;
          break;
        case WRIST_LOW :
          m_wristTargetDegrees = m_wristAngleLow;
          break;
        case WRIST_MID :
          m_wristTargetDegrees = m_wristAngleMid;
          break;
        case WRIST_HIGH :
          m_wristTargetDegrees = m_wristAngleHigh;
          break;
        case WRIST_SHELF :
          m_wristTargetDegrees = m_wristAngleShelf;
          break;
        case WRIST_SCORE :
          m_wristTargetDegrees = m_wristAngleScore;
          break;
      }   
    }

    DataLogManager.log(String.format("%s: TARGET ANGLE %.1f", getSubsystem( ), m_wristTargetDegrees));

    if (WRConsts.kWristCalibrated && moveIsInRange(Math.abs(m_wristTargetDegrees - m_wristCurDegrees)))
    {
      // angle constraint check/soft limit for max and min angle before raising
      if (!moveIsInRange(m_wristTargetDegrees))
      {
        DataLogManager.log(String.format("%s: Target %.1f degrees is OUT OF RANGE! [%.1f, %.1f]", getSubsystem( ),
            m_wristTargetDegrees, m_wristAngleMin, m_wristAngleMax));
        m_wristTargetDegrees = m_wristCurDegrees;
      }

      m_safetyTimer.restart( );

      if (m_wristValid && WRConsts.kWristCalibrated)
        m_wrist.set(ControlMode.MotionMagic, wristDegreesToCounts(m_wristTargetDegrees), DemandType.ArbitraryFeedForward,
            m_wristTotalFF);

      DataLogManager
          .log(String.format("%s: moving: %.1f -> %.1f degrees", getSubsystem( ), m_wristCurDegrees, m_wristTargetDegrees));
    }
    else
    {
      DataLogManager.log(getSubsystem( ) + ": not calibrated");
      if (m_wristValid)
        m_wrist.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public void moveWristAngleExecute( )
  {
    if (m_wristValid && WRConsts.kWristCalibrated)
      m_wrist.set(ControlMode.MotionMagic, wristDegreesToCounts(m_wristTargetDegrees), DemandType.ArbitraryFeedForward,
          m_wristTotalFF);
  }

  public boolean moveWristAngleIsFinished( )
  {
    double errorInDegrees = 0.0;

    errorInDegrees = m_wristTargetDegrees - m_wristCurDegrees;

    if (Math.abs(errorInDegrees) < m_toleranceDegrees)
    {
      if (++m_withinTolerance >= 5)
      {
        m_moveIsFinished = true;
        DataLogManager.log(String.format("%s: move finished - Time: %.3f  |  Cur degrees: %.1f", getSubsystem( ),
            m_safetyTimer.get( ), m_wristCurDegrees));
      }
    }
    else
    {
      m_withinTolerance = 0;
    }

    if (m_safetyTimer.hasElapsed(WRConsts.kMMSafetyTimeout))
    {
      m_moveIsFinished = true;
      DataLogManager.log(getSubsystem( ) + ": Move Safety timer has timed out!");
    }

    if (m_moveIsFinished)
    {
      m_withinTolerance = 0;
      m_safetyTimer.stop( );
    }

    return (m_wristAngle == WristAngle.WRIST_NOCHANGE) ? false : m_moveIsFinished;
  }

  public void moveWristAngleEnd( )
  {
    m_moveIsFinished = false;
    m_withinTolerance = 0;
    m_safetyTimer.stop( );
  }

}
