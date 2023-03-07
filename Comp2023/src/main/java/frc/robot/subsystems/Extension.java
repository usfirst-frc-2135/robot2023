//
// Arm subystem - extension joint
//
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EXConsts;
import frc.robot.Constants.EXConsts.ExtensionLength;
import frc.robot.Constants.EXConsts.ExtensionMode;
import frc.robot.team2135.PhoenixUtil;

//
// Extension subsystem class
//
public class Extension extends SubsystemBase
{
  // Constants
  private static final int                PIDINDEX                = 0;   // PID in use (0-primary, 1-aux)
  private static final int                SLOTINDEX               = 0;   // Use first PID slot

  // Member objects
  private final WPI_TalonFX               m_extension             = new WPI_TalonFX(Constants.Ports.kCANID_Extension);  //extension
  // private final TalonFXSimCollection      m_extensionMotorSim     = new TalonFXSimCollection(m_extension);
  // private final SingleJointedArmSim       m_extensionSim          = new SingleJointedArmSim(DCMotor.getFalcon500(1),
  //     EXConsts.kExtensionGearRatio, 2.0, EXConsts.kForearmLengthMeters, 0.0, Math.PI, false);

  // Mechanism2d
  private final Mechanism2d               m_extensionMech         = new Mechanism2d(3, 3);
  private final MechanismRoot2d           m_extensionRoot         = m_extensionMech.getRoot("extension", 1.5, 2);
  private MechanismLigament2d             m_extensionLigament     =
      m_extensionRoot.append(new MechanismLigament2d("extension", 1, 0, 6, new Color8Bit(Color.kBlue)));

  private boolean                         m_extensionValid;              // Health indicator for extension Talon 
  private double                          m_extensionLengthOffset = 0.0; // CANCoder length measured at reference point

  //Devices and simulation objs
  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits   = new SupplyCurrentLimitConfiguration(true,
      EXConsts.kSupplyCurrentLimit, EXConsts.kSupplyTriggerCurrent, EXConsts.kSupplyTriggerTime);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits   = new StatorCurrentLimitConfiguration(true,
      EXConsts.kStatorCurrentLimit, EXConsts.kStatorTriggerCurrent, EXConsts.kStatorTriggerTime);

  // Declare module variables
  private int                             m_velocity              = EXConsts.kExtensionMMVelocity;        // motion magic velocity
  private int                             m_acceleration          = EXConsts.kExtensionMMAcceleration;    // motion magic acceleration
  private int                             m_sCurveStrength        = EXConsts.kExtensionMMSCurveStrength;  // motion magic S curve smoothing
  private double                          m_pidKf                 = EXConsts.kExtensionPidKf;             // PID force constant
  private double                          m_pidKp                 = EXConsts.kExtensionPidKp;             // PID proportional
  private double                          m_pidKi                 = EXConsts.kExtensionPidKi;             // PID integral
  private double                          m_pidKd                 = EXConsts.kExtensionPidKd;             // PID derivative
  private int                             m_extensionAllowedError = EXConsts.kExtensionAllowedError;      // PID allowable closed loop error
  private double                          m_toleranceInches       = EXConsts.kExtensionToleranceInches;  // PID tolerance in inches
  private double                          m_arbitraryFF           = EXConsts.kExtensionArbitraryFF;       // Arbitrary Feedfoward (elevators and arms)

  private double                          m_extensionLengthStow   = EXConsts.kExtensionLengthStow;         // extension Stow length
  private double                          m_extensionLengthIdle   = EXConsts.kExtensionLengthIdle;         // extension Stow length
  private double                          m_extensionLengthLow    = EXConsts.kExtensionLengthScoreLow;     // low-peg scoring length   
  private double                          m_extensionLengthMid    = EXConsts.kExtensionLengthScoreMid;     // mid-peg scoring length
  private double                          m_extensionLengthHigh   = EXConsts.kExtensionLengthScoreHigh;    // high-peg scoring length
  private double                          m_extensionMinLength    = EXConsts.kExtensionLengthMin;          // minimum extension allowable length
  private double                          m_extensionMaxLength    = EXConsts.kExtensionLengthMax;          // maximum extension allowable length

  private double                          m_stickDeadband         = Constants.kStickDeadband;         // joystick deadband
  private ExtensionMode                   m_extensionMode         = ExtensionMode.EXTENSION_INIT;             // Mode active with joysticks

  private boolean                         m_extensionDebug        = true;   // DEBUG flag to disable/enable extra logging calls

  private double                          m_extensionTargetInches = 0.0;    // Target length in inches
  private double                          m_extensionCurInches    = 0.0;    // Current length in inches
  private int                             m_withinTolerance       = 0;      // Counter for consecutive readings within tolerance

  private Timer                           m_safetyTimer           = new Timer( ); // Safety timer for use in extension
  private double                          m_safetyTimeout;                // Seconds that the timer ran before stopping

  private int                             maxVelocity;

  // Constructor
  public Extension( )
  {
    setName("Extension");
    setSubsystem("Extension");

    m_extensionValid = PhoenixUtil.getInstance( ).talonFXInitialize(m_extension, "extension");

    if (m_extensionValid)
      extensionTalonInitialize(m_extension, EXConsts.kInvertMotor);

    initSmartDashboard( );

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    if (m_extensionValid)
    {
      int curCounts = (int) m_extension.getSelectedSensorPosition(0);

      if (RobotState.isDisabled( ) && (curCounts < 0))
      {
        curCounts = 0;
        m_extension.setSelectedSensorPosition(curCounts, PIDINDEX, Constants.kCANTimeoutMs);
        m_extensionTargetInches = extensionCountsToInches(curCounts);
      }

      if (m_extensionDebug)
      {
        int curVelocity = (int) m_extension.getSelectedSensorVelocity(0);
        maxVelocity = (maxVelocity > curVelocity) ? maxVelocity : curVelocity;

        SmartDashboard.putNumber("EX_maxVelocity", maxVelocity);
        SmartDashboard.putNumber("EX_curVelocity", curVelocity);
        SmartDashboard.putNumber("EX_curCounts", curCounts);
      }

      m_extensionCurInches = extensionCountsToInches(curCounts);
      SmartDashboard.putNumber("EX_curInches", m_extensionCurInches);
      m_extensionLigament.setLength(m_extensionCurInches);

      double currentDraw = m_extension.getStatorCurrent( );
      SmartDashboard.putNumber("EX_currentDraw", currentDraw);
    }
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input motor voltage from the motor setting
    // m_extensionMotorSim.setBusVoltage(RobotController.getInputVoltage( ));
    // m_extensionSim.setInput(m_extensionMotorSim.getMotorOutputLeadVoltage( ));

    // update for 20 msec loop
    // m_extensionSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    // m_extensionMotorSim
    //     .setIntegratedSensorRawPosition(extensionInchesToCounts(Units.radiansToDegrees(m_extensionSim.getAngleRads( ))));
    // m_extensionMotorSim
    //     .setIntegratedSensorVelocity(extensionInchesToCounts(Units.radiansToDegrees(m_extensionSim.getVelocityRadPerSec( ))));

    // SimBattery estimates loaded battery voltages

  }

  private void initSmartDashboard( )
  {
    SmartDashboard.putBoolean("HL_validEX", m_extensionValid);

    // Initialize Variables
    SmartDashboard.putNumber("EX_velocity", m_velocity);
    SmartDashboard.putNumber("EX_acceleration", m_acceleration);
    SmartDashboard.putNumber("EX_sCurveStrength", m_sCurveStrength);
    SmartDashboard.putNumber("EX_pidKf", m_pidKf);
    SmartDashboard.putNumber("EX_pidKp", m_pidKp);
    SmartDashboard.putNumber("EX_pidKi", m_pidKi);
    SmartDashboard.putNumber("EX_pidKd", m_pidKd);

    SmartDashboard.putNumber("EX_stowLength", m_extensionLengthStow);
    SmartDashboard.putNumber("EX_idleLength", m_extensionLengthIdle);
    SmartDashboard.putNumber("EX_lowLength", m_extensionLengthLow);
    SmartDashboard.putNumber("EX_midLength", m_extensionLengthMid);
    SmartDashboard.putNumber("EX_highLength", m_extensionLengthHigh);

    SmartDashboard.putNumber("EX_curInches", m_extensionCurInches);
    SmartDashboard.putNumber("EX_targetInches", m_extensionTargetInches);
    SmartDashboard.putBoolean("EX_calibrated", EXConsts.kExtensionCalibrated);
    SmartDashboard.putBoolean("EX_normalMode", !m_extensionDebug);

    // post the mechanism to the dashboard
    SmartDashboard.putData("ExtensionMech", m_extensionMech);
  }

  public void initialize( )
  {
    double curEXCounts = 0.0;

    DataLogManager.log(getSubsystem( ) + ": Subsystem initialized!");

    setExtensionStopped( );

    if (m_extensionValid)
      curEXCounts = m_extension.getSelectedSensorPosition(0);
    m_extensionCurInches = extensionCountsToInches((int) curEXCounts);
    m_extensionTargetInches = m_extensionCurInches;
    DataLogManager.log(String.format("%s: Init Target Inches: %.1f", getSubsystem( ), m_extensionTargetInches));
  }

  private int extensionInchesToCounts(double inches)
  {
    return (int) (inches / EXConsts.kExtensionInchesPerCount);
  }

  private double extensionCountsToInches(int counts)
  {
    return counts * EXConsts.kExtensionInchesPerCount;
  }

  public boolean moveIsInRange(double inches)
  {
    return inches > m_extensionMinLength && inches < m_extensionMaxLength;
  }

  private void extensionTalonInitialize(WPI_TalonFX motor, boolean inverted)
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
    motor.configAllowableClosedloopError(SLOTINDEX, m_extensionAllowedError, Constants.kLongCANTimeoutMs);
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

  public void moveExtensionWithJoystick(XboxController joystick)
  {
    double xExtensionValue = 0.0;
    double motorOutput = 0.0;
    double manualSpeedMax = EXConsts.kExtensionSpeedMaxManual;

    xExtensionValue = -joystick.getRightX( );
    if (xExtensionValue > -m_stickDeadband && xExtensionValue < m_stickDeadband)
    {
      if (m_extensionMode != ExtensionMode.EXTENSION_STOPPED)
        DataLogManager.log(getSubsystem( ) + ": move Stopped");
      m_extensionMode = ExtensionMode.EXTENSION_STOPPED;
    }
    else
    {
      // If joystick is above a value, extension will move up
      if (xExtensionValue > m_stickDeadband)
      {
        if (m_extensionMode != ExtensionMode.EXTENSION_IN)
          DataLogManager.log(getSubsystem( ) + ": move Out");
        m_extensionMode = ExtensionMode.EXTENSION_IN;

        xExtensionValue -= m_stickDeadband;
        xExtensionValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (xExtensionValue * Math.abs(xExtensionValue));
      }
      // If joystick is below a value, extension will move down
      else if (xExtensionValue < -m_stickDeadband)
      {
        if (m_extensionMode != ExtensionMode.EXTENSION_OUT)
          DataLogManager.log(getSubsystem( ) + ": move In");
        m_extensionMode = ExtensionMode.EXTENSION_OUT;

        xExtensionValue += m_stickDeadband;
        xExtensionValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (xExtensionValue * Math.abs(xExtensionValue));
      }
    }

    if (!moveIsInRange(m_extensionCurInches))
    {
      DataLogManager.log(getSubsystem( ) + ": move OUT OF RANGE!");
      motorOutput = 0.0;
    }

    if (m_extensionValid)
      m_extension.set(ControlMode.PercentOutput, motorOutput);
  }

  public void setExtensionStopped( )
  {
    DataLogManager.log(getSubsystem( ) + ": now STOPPED");

    if (m_extensionValid)
      m_extension.set(ControlMode.PercentOutput, 0.0);
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveExtensionLengthInit(ExtensionLength length)
  {
    if (m_extensionDebug)
    {
      m_velocity = (int) SmartDashboard.getNumber("EX_velocity", m_velocity);
      m_acceleration = (int) SmartDashboard.getNumber("EX_acceleration", m_acceleration);
      m_sCurveStrength = (int) SmartDashboard.getNumber("EX_sCurveStrength", m_sCurveStrength);
      m_pidKf = SmartDashboard.getNumber("EX_pidKf", m_pidKf);
      m_pidKp = SmartDashboard.getNumber("EX_pidKp", m_pidKp);
      m_pidKi = SmartDashboard.getNumber("EX_pidKi", m_pidKi);
      m_pidKd = SmartDashboard.getNumber("EX_pidKd", m_pidKd);

      m_extension.configMotionCruiseVelocity(m_velocity);
      m_extension.configMotionAcceleration(m_acceleration);
      m_extension.configMotionSCurveStrength(m_sCurveStrength);
      m_extension.config_kF(SLOTINDEX, m_pidKf);
      m_extension.config_kP(SLOTINDEX, m_pidKp);
      m_extension.config_kI(SLOTINDEX, m_pidKi);
      m_extension.config_kD(SLOTINDEX, m_pidKd);

      m_extensionLengthStow = SmartDashboard.getNumber("EX_stowLength", m_extensionLengthStow);
      m_extensionLengthIdle = SmartDashboard.getNumber("EX_idleLength", m_extensionLengthIdle);
      m_extensionLengthLow = SmartDashboard.getNumber("EX_lowLength", m_extensionLengthLow);
      m_extensionLengthMid = SmartDashboard.getNumber("EX_midLength", m_extensionLengthMid);
      m_extensionLengthHigh = SmartDashboard.getNumber("EX_highLength", m_extensionLengthHigh);
    }

    switch (length)
    {
      case EXTENSION_NOCHANGE : // Do not change from current level!
        m_extensionTargetInches = m_extensionCurInches;
        if (m_extensionTargetInches < 0.25)
          m_extensionTargetInches = 0.25;
        break;
      case EXTENSION_STOW :
        m_extensionTargetInches = m_extensionLengthStow;
        break;
      case EXTENSION_IDLE :
        m_extensionTargetInches = m_extensionLengthIdle;
        break;
      case EXTENSION_LOW :
        m_extensionTargetInches = m_extensionLengthLow;
        break;
      case EXTENSION_MID :
        m_extensionTargetInches = m_extensionLengthMid;
        break;
      case EXTENSION_HIGH :
        m_extensionTargetInches = m_extensionLengthHigh;
        break;
      case EXTENSION_SHELF :
        m_extensionTargetInches = m_extensionLengthHigh;
        break;
      default :
        DataLogManager.log(String.format("%s: requested length is invalid - %.1f", getSubsystem( ), length));
        return;
    }

    DataLogManager.log(String.format("%s: TARGET DEGREES %.1f", getSubsystem( ), m_extensionTargetInches));

    if (EXConsts.kExtensionCalibrated && moveIsInRange(Math.abs(m_extensionTargetInches - m_extensionCurInches)))
    {
      // length constraint check/soft limit for max and min length before raising
      if (!moveIsInRange(m_extensionTargetInches))
      {
        DataLogManager.log(String.format("%s: Target %.1f inches is OUT OF RANGE! [%.1f, %.1f]", getSubsystem( ),
            m_extensionTargetInches, m_extensionMinLength, m_extensionMaxLength));
        m_extensionTargetInches = m_extensionCurInches;
      }

      // Start the safety timer
      m_safetyTimeout = 1.8;
      m_safetyTimer.reset( );
      m_safetyTimer.start( );

      if (m_extensionValid)
        m_extension.set(ControlMode.MotionMagic, extensionInchesToCounts(m_extensionTargetInches));

      DataLogManager.log(String.format("%s: moving: %.1f -> %.1f inches | counts %d -> %d", getSubsystem( ), m_extensionCurInches,
          m_extensionTargetInches, m_extensionCurInches, m_extensionTargetInches));
    }
    else
    {
      DataLogManager.log(getSubsystem( ) + ": not calibrated");
      if (m_extensionValid)
        m_extension.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public void moveExtensionLengthExecute( )
  {
    if (m_extensionValid && EXConsts.kExtensionCalibrated)
      m_extension.set(ControlMode.MotionMagic, extensionInchesToCounts(m_extensionTargetInches), DemandType.ArbitraryFeedForward,
          m_arbitraryFF * Math.sin(Units.degreesToRadians((m_extensionCurInches))));
  }

  public boolean moveExtensionLengthIsFinished( )
  {
    boolean isFinished = false;
    double errorInInches = 0.0;

    errorInInches = m_extensionTargetInches - m_extensionCurInches;

    if (Math.abs(errorInInches) < m_toleranceInches)
    {
      if (++m_withinTolerance >= 5)
      {
        isFinished = true;
        DataLogManager.log(String.format("%s: move finished - Time: %.3f  |  Cur inches: %.1f", getSubsystem( ),
            m_safetyTimer.get( ), m_extensionCurInches));
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
