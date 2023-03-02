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

import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.Constants.WRConsts;
import frc.robot.Constants.WRConsts.WristAngle;
import frc.robot.Constants.WRConsts.WristMode;
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
      WRConsts.kWristGearRatio, 2.0, WRConsts.kGripperLengthMeters, 0, Math.PI, false);

  private final Mechanism2d               m_wristMech           = new Mechanism2d(3, 3);
  private final MechanismRoot2d           m_wristRoot           = m_wristMech.getRoot("wrist", 1.5, 2);
  private final MechanismLigament2d       m_wristLigament       =
      m_wristRoot.append(new MechanismLigament2d("wrist", 0.5, 0, 6, new Color8Bit(Color.kPurple)));

  private boolean                         m_wristValid;                // Health indicator for wrist Talon 
  private double                          m_wristAngleOffset;          // CANCoder angle measured at reference point

  //Devices and simulation objs
  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true,
      WRConsts.kSupplyCurrentLimit, WRConsts.kSupplyTriggerCurrent, WRConsts.kSupplyTriggerTime);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true,
      WRConsts.kStatorCurrentLimit, WRConsts.kStatorTriggerCurrent, WRConsts.kStatorTriggerTime);

  // Declare module variables
  private int                             m_velocity            = WRConsts.kWristMMVelocity;         // motion magic velocity
  private int                             m_acceleration        = WRConsts.kWristMMAcceleration;     // motion magic acceleration
  private int                             m_sCurveStrength      = WRConsts.kWristMMSCurveStrength;   // motion magic S curve smoothing
  private double                          m_pidKf               = WRConsts.kWristPidKf;              // PID force constant
  private double                          m_pidKp               = WRConsts.kWristPidKp;              // PID proportional
  private double                          m_pidKi               = WRConsts.kWristPidKi;              // PID integral
  private double                          m_pidKd               = WRConsts.kWristPidKd;              // PID derivative
  private int                             m_wristAllowedError   = WRConsts.kWristAllowedError;       // PID allowable closed loop error
  private double                          m_toleranceDegrees    = WRConsts.kWristToleranceDegrees;   // PID tolerance in Degrees
  private double                          m_arbitraryFF         = WRConsts.kWristArbitraryFF;        // Arbitrary Feedfoward (elevators and arms))

  private double                          m_wristAngleStow      = WRConsts.kWristAngleStow;          // wrist Stow angle
  private double                          m_wristAngleLow       = WRConsts.kWristAngleScoreLow;      // low-peg scoring angle   
  private double                          m_wristAngleMid       = WRConsts.kWristAngleScoreMid;      // mid-peg scoring angle
  private double                          m_wristAngleHigh      = WRConsts.kWristAngleScoreHigh;     // high-peg scoring angle
  private double                          m_wristMinAngle       = WRConsts.kWristMinAngle;           // minimum wrist allowable angle
  private double                          m_wristMaxAngle       = WRConsts.kWristMaxAngle;           // maximum wrist allowable angle

  private double                          m_stickDeadband       = Constants.kStickDeadband;          // joystick deadband
  private WristMode                       m_wristMode           = WristMode.WRIST_INIT;              // Mode active with joysticks

  private boolean                         m_wristDebug          = false;  // DEBUG flag to disable/enable extra logging calls

  private double                          m_wristTargetDegrees  = 0.0;    // Target angle in degrees
  private double                          m_wristCurDegrees     = 0.0;    // Current angle in degrees
  private int                             m_withinTolerance     = 0;      // Counter for consecutive readings within tolerance

  private Timer                           m_safetyTimer         = new Timer( ); // Safety timer for use in wrist
  private double                          m_safetyTimeout;                // Seconds that the timer ran before stopping

  private int                             maxVelocity;

  // Constructor
  public Wrist( )
  {
    setName("Wrist");
    setSubsystem("Wrist");

    m_wristValid = PhoenixUtil.getInstance( ).talonFXInitialize(m_wrist, "wrist");

    if (m_wristValid)
      wristTalonInitialize(m_wrist, WRConsts.kInvertMotor);

    m_wristAngleOffset = (Constants.isComp) ? WRConsts.kCompWristOffset : WRConsts.kBetaWristOffset;
    m_wristCANCoder.configFactoryDefault( );
    m_wristCANCoder.configAllSettings(CTREConfigs.wristCancoderConfig( ));
    m_wristCANCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
    m_wristCANCoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);
    resetToAbsolute( );

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
      m_wristLigament.setAngle(m_wristCurDegrees);
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

    SmartDashboard.putNumber("WR_stowAngle", m_wristAngleStow);
    SmartDashboard.putNumber("WR_scoreAngleLow", m_wristAngleLow);
    SmartDashboard.putNumber("WR_scoreAngleMid", m_wristAngleMid);
    SmartDashboard.putNumber("WR_scoreAngleHigh", m_wristAngleHigh);

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

  public void resetToAbsolute( )
  {
    double absolutePosition = ((WRConsts.kWristCANCoderAbsInvert) ? -1.0 : 1.0)
        * Conversions.degreesToFalcon(getCanCoder( ).getDegrees( ) - m_wristAngleOffset, WRConsts.kWristGearRatio);
    m_wrist.setSelectedSensorPosition(absolutePosition);
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
    return degrees > m_wristMinAngle && degrees < m_wristMaxAngle;
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
    double yWristValue = 0.0;
    double motorOutput = 0.0;
    double manualSpeedMax = WRConsts.kWristSpeedMaxManual;

    yWristValue = -joystick.getRightY( );
    if (yWristValue > -m_stickDeadband && yWristValue < m_stickDeadband)
    {
      if (m_wristMode != WristMode.WRIST_STOPPED)
        DataLogManager.log(getSubsystem( ) + ": move Stopped");
      m_wristMode = WristMode.WRIST_STOPPED;
    }
    else
    {
      // If joystick is above a value, wrist will move up
      if (yWristValue > m_stickDeadband)
      {
        if (m_wristMode != WristMode.WRIST_UP)
          DataLogManager.log(getSubsystem( ) + ": move Up");
        m_wristMode = WristMode.WRIST_UP;

        yWristValue -= m_stickDeadband;
        yWristValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yWristValue * Math.abs(yWristValue));
      }
      // If joystick is below a value, wrist will move down
      else if (yWristValue < -m_stickDeadband)
      {
        if (m_wristMode != WristMode.WRIST_DOWN)
          DataLogManager.log(getSubsystem( ) + " : move Down");
        m_wristMode = WristMode.WRIST_DOWN;

        yWristValue += m_stickDeadband;
        yWristValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yWristValue * Math.abs(yWristValue));
      }
    }

    if (!moveIsInRange(m_wristCurDegrees))
    {
      DataLogManager.log(getSubsystem( ) + ": move OUT OF RANGE!");
      motorOutput = 0.0;
    }

    if (m_wristValid)
      m_wrist.set(ControlMode.PercentOutput, motorOutput);
  }

  public void setWristStopped( )
  {
    DataLogManager.log(getSubsystem( ) + ": now STOPPED");

    if (m_wristValid)
      m_wrist.set(ControlMode.PercentOutput, 0.0);
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

      m_wristAngleStow = SmartDashboard.getNumber("WR_stowAngle", m_wristAngleStow);
      m_wristAngleLow = SmartDashboard.getNumber("WR_scoreAngleLow", m_wristAngleLow);
      m_wristAngleMid = SmartDashboard.getNumber("WR_scoreAngleMid", m_wristAngleMid);
      m_wristAngleHigh = SmartDashboard.getNumber("WR_scoreAngleHigh", m_wristAngleHigh);
    }

    switch (angle) // Do not change from current level!
    {
      case WRIST_NOCHANGE : // Do not change from current level!
        m_wristTargetDegrees = m_wristCurDegrees;
        if (m_wristTargetDegrees < 0.25)
          m_wristTargetDegrees = 0.25;
        break;
      case WRIST_STOW :
        m_wristTargetDegrees = m_wristAngleStow;
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
        m_wristTargetDegrees = m_wristAngleHigh;
        break;
      default :
        DataLogManager.log(String.format("%s: requested angle is invalid - %.1f", getSubsystem( ), angle));
        return;
    }

    DataLogManager.log(String.format("%s: TARGET ANGLE %.1f", getSubsystem( ), m_wristTargetDegrees));

    if (WRConsts.kWristCalibrated)
      if (WRConsts.kWristCalibrated && moveIsInRange(Math.abs(m_wristTargetDegrees - m_wristCurDegrees)))
      {
        // angle constraint check/soft limit for max and min angle before raising
        if (!moveIsInRange(m_wristTargetDegrees))
        {
          DataLogManager.log(String.format("%s: Target %.1f degrees is OUT OF RANGE! [%.1f, %.1f]", getSubsystem( ),
              m_wristTargetDegrees, m_wristMinAngle, m_wristMaxAngle));
          m_wristTargetDegrees = m_wristCurDegrees;
        }

        // Start the safety timer
        m_safetyTimeout = 1.8;
        m_safetyTimer.reset( );
        m_safetyTimer.start( );

        if (m_wristValid)
          m_wrist.set(ControlMode.MotionMagic, wristDegreesToCounts(m_wristTargetDegrees));

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
          m_arbitraryFF * Math.sin(Units.degreesToRadians((m_wristCurDegrees))));
  }

  public boolean moveWristAngleIsFinished( )
  {
    boolean isFinished = false;
    double errorInDegrees = 0.0;

    errorInDegrees = m_wristTargetDegrees - m_wristCurDegrees;

    if (Math.abs(errorInDegrees) < m_toleranceDegrees)
    {
      if (++m_withinTolerance >= 5)
      {
        isFinished = true;
        DataLogManager.log(String.format("%s: move finished - Time: %.3f  |  Cur degrees: %.1f", getSubsystem( ),
            m_safetyTimer.get( ), m_wristCurDegrees));
      }
    }
    else
    {
      m_withinTolerance = 0;
    }

    if (m_safetyTimer.get( ) >= m_safetyTimeout)
    {
      isFinished = true;
      DataLogManager.log(getSubsystem( ) + ": Move Safety timer has timed out!");
    }

    if (isFinished)
    {
      m_withinTolerance = 0;
      m_safetyTimer.stop( );
    }

    return isFinished;
  }
}
