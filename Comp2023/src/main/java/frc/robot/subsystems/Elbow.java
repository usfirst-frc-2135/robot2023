//
// Arm subystem - elbow joint
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
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.ELConsts.ElbowAngle;
import frc.robot.Constants.ELConsts.ElbowMode;
import frc.robot.Constants.Falcon500;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.CTREConfigs;
import frc.robot.team2135.PhoenixUtil;

//
// Elbow subsystem class
//
public class Elbow extends SubsystemBase
{
  // Constants
  private static final int                PIDINDEX              = 0;   // PID in use (0-primary, 1-aux)
  private static final int                SLOTINDEX             = 0;   // Use first PID slot

  // Member objects
  private final WPI_TalonFX               m_elbow               = new WPI_TalonFX(Constants.Ports.kCANID_Elbow);  //elbow
  private final CANCoder                  m_elbowCANCoder       = new CANCoder(Constants.Ports.kCANID_ELCANCoder);
  private final TalonFXSimCollection      m_elbowMotorSim       = new TalonFXSimCollection(m_elbow);
  private final SingleJointedArmSim       m_elbowSim            = new SingleJointedArmSim(DCMotor.getFalcon500(1),
      ELConsts.kElbowGearRatio, 2.0, ELConsts.kForearmLengthMeters, 0.0, Math.PI, true);
  private final Mechanism2d               m_elbowMech           = new Mechanism2d(3, 3);
  private final MechanismLigament2d       m_elbowLigament;

  private boolean                         m_elbowValid;                // Health indicator for elbow Talon 
  private double                          m_elbowAngleOffset;

  //Devices and simulation objs
  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true,
      Falcon500.kSupplyCurrentLimit, Falcon500.kSupplyTriggerCurrent, Falcon500.kSupplyTriggerTime);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true,
      Falcon500.kStatorCurrentLimit, Falcon500.kStatorTriggerCurrent, Falcon500.kStatorTriggerTime);

  // Declare module variables
  private int                             m_velocity            = ELConsts.kElbowMMVelocity;        // motion magic velocity
  private int                             m_acceleration        = ELConsts.kElbowMMAcceleration;    // motion magic acceleration
  private int                             m_sCurveStrength      = ELConsts.kElbowMMSCurveStrength;  // motion magic S curve smoothing
  private double                          m_pidKf               = ELConsts.kElbowPidKf;             // PID force constant
  private double                          m_pidKp               = ELConsts.kElbowPidKp;             // PID proportional
  private double                          m_pidKi               = ELConsts.kElbowPidKi;             // PID integral
  private double                          m_pidKd               = ELConsts.kElbowPidKd;             // PID derivative
  private int                             m_elbowAllowedError   = ELConsts.kElbowAllowedError;      // PID allowable closed loop error
  private double                          m_toleranceDegrees    = ELConsts.kElbowToleranceDegrees;  // PID tolerance in inches
  private double                          m_arbitraryFF         = ELConsts.kElbowArbitraryFF;       // Arbitrary Feedfoward (elevators and arms)

  private double                          m_elbowStowangle      = ELConsts.kElbowAngleStow;         // elbow Stow angle
  private double                          m_lowScoreangle       = ELConsts.kElbowAngleScoreLow;     // low-peg scoring angle   
  private double                          m_midScoreangle       = ELConsts.kElbowAngleScoreMid;     // mid-peg scoring angle
  private double                          m_highScoreangle      = ELConsts.kElbowAngleScoreHigh;    // high-peg scoring angle
  private double                          m_elbowMinangle       = ELConsts.kElbowAnglewMin;         // minimum elbow allowable angle
  private double                          m_elbowMaxangle       = ELConsts.kElbowAngleMax;          // maximum elbow allowable angle

  private double                          m_stickDeadband       = Constants.kStickDeadband;         // joystick deadband
  private ElbowMode                       m_elbowMode           = ElbowMode.ELBOW_INIT;             // Mode active with joysticks

  private int                             m_elbowDebug          = 1; // DEBUG flag to disable/enable extra logging calls

  private boolean                         m_calibrated          = true;  // Indicates whether the elbow has been calibrated
  private double                          m_elbowTargetDegrees  = 0.0;    // Target angle in degrees
  private double                          m_elbowCurDegrees     = 0.0;    // Current angle in degrees
  private int                             m_withinTolerance     = 0;      // Counter for consecutive readings within tolerance

  private Timer                           m_safetyTimer         = new Timer( ); // Safety timer for use in elbow
  private double                          m_safetyTimeout;                // Seconds that the timer ran before stopping

  private int                             maxVelocity;

  // Constructor
  public Elbow( )
  {
    setName("Elbow");
    setSubsystem("Elbow");

    m_elbowValid = PhoenixUtil.getInstance( ).talonFXInitialize(m_elbow, "elbow");

    SmartDashboard.putBoolean("HL_validEL", m_elbowValid);

    // Initialize Variables
    SmartDashboard.putNumber("EL_velocity", m_velocity);
    SmartDashboard.putNumber("EL_acceleration", m_acceleration);
    SmartDashboard.putNumber("EL_sCurveStrength", m_sCurveStrength);
    SmartDashboard.putNumber("EL_pidKf", m_pidKf);
    SmartDashboard.putNumber("EL_pidKp", m_pidKp);
    SmartDashboard.putNumber("EL_pidKi", m_pidKi);
    SmartDashboard.putNumber("EL_pidKd", m_pidKd);

    SmartDashboard.putNumber("EL_stowangle", m_elbowStowangle);
    SmartDashboard.putNumber("EL_scoreAngleLow", m_lowScoreangle);
    SmartDashboard.putNumber("EL_scoreAngleMid", m_midScoreangle);
    SmartDashboard.putNumber("EL_scoreAngleHigh", m_highScoreangle);

    SmartDashboard.putNumber("EL_curDegrees", m_elbowCurDegrees);
    SmartDashboard.putNumber("EL_targetDegrees", m_elbowTargetDegrees);
    SmartDashboard.putBoolean("EL_calibrated", m_calibrated);

    if (m_elbowValid)
      elbowTalonInitialize(m_elbow, ELConsts.kInvertMotor);

    m_elbowAngleOffset = (Constants.isComp) ? ELConsts.kCompElbowOffset : ELConsts.kBetaElbowOffset;
    m_elbowCANCoder.configFactoryDefault( );
    m_elbowCANCoder.configAllSettings(CTREConfigs.elbowCancoderConfig( ));
    m_elbowCANCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
    m_elbowCANCoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);
    resetToAbsolute( );

    // the mechanism root node
    MechanismRoot2d elbowRoot = m_elbowMech.getRoot("elbow", 1.5, 2);

    // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
    // off the root node or another ligament object
    m_elbowLigament = elbowRoot.append(new MechanismLigament2d("elbow", 1, 0, 6, new Color8Bit(Color.kBlue)));

    // post the mechanism to the dashboard
    SmartDashboard.putData("ElbowMech2d", m_elbowMech);

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    if (m_elbowValid)
    {
      int curCounts = (int) m_elbow.getSelectedSensorPosition(0);
      int curVelocity = (int) m_elbow.getSelectedSensorVelocity(0);
      maxVelocity = (maxVelocity > curVelocity) ? maxVelocity : curVelocity;
      SmartDashboard.putNumber("EL_maxVelocity", maxVelocity);
      SmartDashboard.putNumber("EL_curVelocity", curVelocity);
      SmartDashboard.putNumber("EL_curCounts", curCounts);
      m_elbowCurDegrees = elbowCountsToDegrees(curCounts);
      SmartDashboard.putNumber("EL_curDegrees", m_elbowCurDegrees);
      m_elbowLigament.setAngle(elbowCountsToDegrees(curCounts));
    }
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input motor voltage from the motor setting
    m_elbowMotorSim.setBusVoltage(RobotController.getInputVoltage( ));
    m_elbowSim.setInput(m_elbowMotorSim.getMotorOutputLeadVoltage( ));

    // update for 20 msec loop
    m_elbowSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_elbowMotorSim.setIntegratedSensorRawPosition(elbowDegreesToCounts(Units.radiansToDegrees(m_elbowSim.getAngleRads( ))));
    m_elbowMotorSim.setIntegratedSensorVelocity(elbowDegreesToCounts(Units.radiansToDegrees(m_elbowSim.getVelocityRadPerSec( ))));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elbowSim.getCurrentDrawAmps( )));
  }

  public void initialize( )
  {
    double curELCounts = 0.0;

    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");

    setElbowStopped( );

    if (m_elbowValid)
      curELCounts = m_elbow.getSelectedSensorPosition(0);
    m_elbowCurDegrees = elbowCountsToDegrees((int) curELCounts);
    m_elbowTargetDegrees = m_elbowCurDegrees;
    DataLogManager.log(getSubsystem( ) + ": Init Target Degrees: " + m_elbowTargetDegrees);
  }

  public Rotation2d getCanCoder( )
  {
    return Rotation2d.fromDegrees(m_elbowCANCoder.getAbsolutePosition( ));
  }

  public void resetToAbsolute( )
  {
    double absolutePosition = ((ELConsts.kElbowCANCoderAbsInvert) ? -1.0 : 1.0)
        * Conversions.degreesToFalcon(getCanCoder( ).getDegrees( ) - m_elbowAngleOffset, ELConsts.kElbowGearRatio);
    m_elbow.setSelectedSensorPosition(absolutePosition);
  }

  private int elbowDegreesToCounts(double degrees)
  {
    return (int) (degrees / ELConsts.kElbowDegreesPerCount);
  }

  private double elbowCountsToDegrees(int counts)
  {
    return counts * ELConsts.kElbowDegreesPerCount;
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
    motor.configAllowableClosedloopError(SLOTINDEX, m_elbowAllowedError, Constants.kLongCANTimeoutMs);
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

  public void moveElbowWithJoystick(XboxController joystick)
  {
    double yElbowValue = 0.0;
    double motorOutput = 0.0;
    double manualSpeedMax = ELConsts.kElbowSpeedMaxManual;

    yElbowValue = -joystick.getLeftY( );
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

    if (m_elbowCurDegrees < m_elbowMinangle || m_elbowCurDegrees > m_elbowMaxangle)
      DataLogManager.log(getSubsystem( ) + ": Elbow movement OUT OF RANGE!");

    if (m_elbowValid)
      m_elbow.set(ControlMode.PercentOutput, motorOutput);
  }

  public void setElbowStopped( )
  {
    DataLogManager.log(getSubsystem( ) + ": Elbow now STOPPED");

    if (m_elbowValid)
      m_elbow.set(ControlMode.PercentOutput, 0.0);
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveElbowAngleInit(ElbowAngle angle)
  {
    if (m_elbowDebug != 0)
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

    switch (angle)
    {
      case ELBOW_NOCHANGE : // Do not change from current level!
        m_elbowTargetDegrees = m_elbowCurDegrees;
        if (m_elbowTargetDegrees < 0.25)
          m_elbowTargetDegrees = 0.25;
        break;
      case ELBOW_STOW :
        m_elbowTargetDegrees = SmartDashboard.getNumber("EL_stowangle", m_elbowStowangle);
        DataLogManager.log("ELBOW TARGET ANGLE: " + m_elbowTargetDegrees);
        break;
      case ELBOW_LOW :
        m_elbowTargetDegrees = SmartDashboard.getNumber("EL_lowScoreangle", m_lowScoreangle);
        DataLogManager.log("ELBOW TARGET ANGLE: " + m_elbowTargetDegrees);
        break;
      case ELBOW_MID :
        m_elbowTargetDegrees = SmartDashboard.getNumber("EL_midScoreangle", m_midScoreangle);
        DataLogManager.log("ELBOW TARGET ANGLE: " + m_elbowTargetDegrees);
        break;
      case ELBOW_HIGH :
        m_elbowTargetDegrees = SmartDashboard.getNumber("EL_highScoreangle", m_highScoreangle);
        DataLogManager.log("ELBOW TARGET ANGLE: " + m_elbowTargetDegrees);
        break;
      default :
        DataLogManager.log(getSubsystem( ) + ": requested angle is invalid - " + angle);
        return;
    }

    if (m_calibrated)
    {
      // angle constraint check/soft limit for max and min angle before raising
      if (m_elbowTargetDegrees < m_elbowMinangle)
      {
        DataLogManager.log("Target " + String.format("%.1f", m_elbowTargetDegrees) + " degrees is limited by "
            + String.format("%.1f", m_elbowMinangle) + " degrees");
        m_elbowTargetDegrees = m_elbowMinangle;
      }

      if (m_elbowTargetDegrees > m_elbowMaxangle)
      {
        DataLogManager.log("Target " + String.format("%.1f", m_elbowTargetDegrees) + " degrees is limited by "
            + String.format("%.1f", m_elbowMaxangle) + " degrees");
        m_elbowTargetDegrees = m_elbowMaxangle;
      }

      // Start the safety timer
      m_safetyTimeout = 1.8;
      m_safetyTimer.reset( );
      m_safetyTimer.start( );

      m_elbow.set(ControlMode.MotionMagic, elbowDegreesToCounts(m_elbowTargetDegrees));

      DataLogManager.log("Elbow moving: " + String.format("%.1f", m_elbowCurDegrees) + " -> "
          + String.format("%.1f", m_elbowTargetDegrees) + " degrees  |  counts " + elbowDegreesToCounts(m_elbowCurDegrees)
          + " -> " + elbowDegreesToCounts(m_elbowTargetDegrees));
    }
    else
    {
      DataLogManager.log("Elbow is not calibrated");
      if (m_elbowValid)
        m_elbow.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public void moveElbowAngleExecute( )
  {
    if (m_elbowValid && m_calibrated)
      m_elbow.set(ControlMode.MotionMagic, elbowDegreesToCounts(m_elbowTargetDegrees), DemandType.ArbitraryFeedForward,
          m_arbitraryFF * Math.sin(Units.degreesToRadians((m_elbowCurDegrees))));
  }

  public boolean moveElbowAngleIsFinished( )
  {
    boolean isFinished = false;
    double errorInDegrees = 0.0;

    errorInDegrees = m_elbowTargetDegrees - m_elbowCurDegrees;

    if (Math.abs(errorInDegrees) < m_toleranceDegrees)
    {
      if (++m_withinTolerance >= 5)
      {
        isFinished = true;
        DataLogManager.log("Elbow move finished - Time: " + String.format("%.3f", m_safetyTimer.get( )) + "  |  Cur degrees: "
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
      DataLogManager.log("Elbow Move Safety timer has timed out!");
    }

    if (isFinished)
    {
      m_withinTolerance = 0;
      m_safetyTimer.stop( );
    }

    return isFinished;
  }
}
