//
// Arm subystem - elbow joint
//
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.ELConsts.ElbowAngle;
import frc.robot.Constants.ELConsts.ElbowMode;
import frc.robot.Constants.EXConsts;
import frc.robot.RobotContainer;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.CTREConfigs;
import frc.robot.team2135.PhoenixUtil;

//
// Elbow subsystem class
//
public class Elbow extends SubsystemBase
{
  // Constants
  private static final int           PIDINDEX             = 0;   // PID in use (0-primary, 1-aux)
  private static final int           SLOTINDEX            = 0;   // Use first PID slot

  // Member objects
  private final WPI_TalonFX          m_elbow              = new WPI_TalonFX(Constants.Ports.kCANID_Elbow);  //elbow
  private final CANCoder             m_elbowCANCoder      = new CANCoder(Constants.Ports.kCANID_ELCANCoder);
  private final TalonFXSimCollection m_elbowMotorSim      = new TalonFXSimCollection(m_elbow);
  private final SingleJointedArmSim  m_elbowSim           = new SingleJointedArmSim(DCMotor.getFalcon500(1), ELConsts.kGearRatio,
      2.0, ELConsts.kForearmLengthMeters, -Math.PI, Math.PI, false);

  // Mechanism2d
  private final Mechanism2d          m_elbowMech          = new Mechanism2d(3, 3);
  private final MechanismRoot2d      m_elbowRoot          = m_elbowMech.getRoot("elbow", 1.5, 2);
  private final MechanismLigament2d  m_elbowLigament      =
      m_elbowRoot.append(new MechanismLigament2d("elbow", 1, 0, 6, new Color8Bit(Color.kBlue)));

  private boolean                    m_elbowValid;                 // Health indicator for elbow Talon 
  private boolean                    m_elbowCCValid;               // Health indicator for elbow CANCoder 

  // Declare module variables
  private ElbowMode                  m_elbowMode          = ElbowMode.ELBOW_INIT;             // Mode active with joysticks

  private ElbowAngle                 m_elbowAngle;                   // Desired elbow angle
  private boolean                    m_moveIsFinished;
  private double                     m_elbowTargetDegrees = 0.0;    // Target angle in degrees
  private double                     m_elbowCurDegrees    = 0.0;    // Current angle in degrees
  private int                        m_withinTolerance    = 0;      // Counter for consecutive readings within tolerance
  private double                     m_elbowTotalFF;

  private Timer                      m_safetyTimer        = new Timer( ); // Safety timer for use in elbow

  // Constructor
  public Elbow( )
  {
    setName("Elbow");
    setSubsystem("Elbow");

    m_elbowValid = PhoenixUtil.getInstance( ).talonFXInitialize(m_elbow, "elbow");

    if (m_elbowValid)
      elbowTalonInitialize(m_elbow, ELConsts.kInvertMotor);

    m_elbowCCValid = PhoenixUtil.getInstance( ).canCoderInitialize(m_elbowCANCoder, "elbow");

    if (m_elbowCCValid)
    {
      m_elbowCANCoder.configAllSettings(CTREConfigs.elbowCancoderConfig( ));
      m_elbowCurDegrees = getCanCoder( ).getDegrees( );

      // Slow status frame updates AFTER getting the absolute position
      m_elbowCANCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
      m_elbowCANCoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);

      DataLogManager.log(getSubsystem( ) + ": Initial degrees " + m_elbowCurDegrees);
      double absolutePosition = Conversions.degreesToFalcon(m_elbowCurDegrees, ELConsts.kGearRatio);

      if (RobotBase.isReal( ))
        m_elbow.setSelectedSensorPosition(absolutePosition);
    }

    m_elbow.configReverseSoftLimitThreshold(Conversions.degreesToFalcon(ELConsts.kAngleMin, ELConsts.kGearRatio),
        Constants.kCANTimeoutMs);
    m_elbow.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);
    m_elbow.configForwardSoftLimitThreshold(Conversions.degreesToFalcon(ELConsts.kAngleMax, ELConsts.kGearRatio),
        Constants.kCANTimeoutMs);
    m_elbow.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);

    initSmartDashboard( );

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
      SmartDashboard.putNumber("EL_targetDegrees", m_elbowTargetDegrees);
      m_elbowLigament.setAngle(m_elbowCurDegrees);

      m_elbowTotalFF = calculateTotalFF( );
      SmartDashboard.putNumber("EL_totalFF", m_elbowTotalFF);

      double currentDraw = m_elbow.getStatorCurrent( );
      SmartDashboard.putNumber("EL_currentDraw", currentDraw);
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

  private void initSmartDashboard( )
  {
    SmartDashboard.putBoolean("HL_validEL", m_elbowValid);

    // Initialize Variables
    SmartDashboard.putNumber("EL_curDegrees", m_elbowCurDegrees);
    SmartDashboard.putNumber("EL_targetDegrees", m_elbowTargetDegrees);
    SmartDashboard.putBoolean("EL_calibrated", ELConsts.kCalibrated);

    // post the mechanism to the dashboard
    SmartDashboard.putData("ElbowMech", m_elbowMech);
  }

  public void initialize( )
  {
    double curELCounts = 0.0;

    DataLogManager.log(getSubsystem( ) + ": Subsystem initialized!");

    setElbowStopped( );

    if (m_elbowValid)
      curELCounts = m_elbow.getSelectedSensorPosition(0);
    m_elbowCurDegrees = elbowCountsToDegrees((int) curELCounts);
    m_elbowTargetDegrees = m_elbowCurDegrees;
    DataLogManager.log(String.format("%s: Init Target Degrees: %.1f", getSubsystem( ), m_elbowTargetDegrees));
  }

  public Rotation2d getCanCoder( )
  {
    return Rotation2d.fromDegrees(m_elbowCANCoder.getAbsolutePosition( ));
  }

  private int elbowDegreesToCounts(double degrees)
  {
    return (int) (degrees / ELConsts.kDegreesPerCount);
  }

  private double elbowCountsToDegrees(int counts)
  {
    return counts * ELConsts.kDegreesPerCount;
  }

  public boolean moveIsInRange(double degrees)
  {
    return (degrees > ELConsts.kAngleMin) && (degrees < ELConsts.kAngleMax);
  }

  public double getAngle( )
  {
    return m_elbowCurDegrees;
  }

  private void elbowTalonInitialize(WPI_TalonFX motor, boolean inverted)
  {
    motor.configFactoryDefault( );
    motor.configAllSettings(CTREConfigs.elbowAngleFXConfig( ));
    motor.setInverted(inverted);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setInverted");
    motor.setNeutralMode(NeutralMode.Brake);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setNeutralMode");
    motor.setSafetyEnabled(false);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setSafetyEnabled");

    motor.enableVoltageCompensation(true);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "enableVoltageCompensation");

    // Configure sensor settings
    motor.setSelectedSensorPosition(0.0);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setSelectedSensorPosition");
    motor.configAllowableClosedloopError(SLOTINDEX, ELConsts.kAllowedError, Constants.kLongCANTimeoutMs);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configAllowableClosedloopError");

    // Configure Magic Motion settings
    motor.selectProfileSlot(SLOTINDEX, PIDINDEX);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "selectProfileSlot");

    motor.set(ControlMode.PercentOutput, 0.0);
  }

  public void moveElbowWithJoystick(XboxController joystick)
  {
    double axisValue = -joystick.getLeftY( );
    boolean outOfRange = false;
    ElbowMode newMode = ElbowMode.ELBOW_STOPPED;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if (axisValue < 0.0)
    {
      if (m_elbowCurDegrees > ELConsts.kAngleMin)
        newMode = ElbowMode.ELBOW_DOWN;
      else
        outOfRange = true;
    }
    else if (axisValue > 0.0)
    {
      if (m_elbowCurDegrees < ELConsts.kAngleMax)
        newMode = ElbowMode.ELBOW_UP;
      else
        outOfRange = true;
    }

    if (outOfRange)
      axisValue = 0.0;

    if (newMode != m_elbowMode)
    {
      m_elbowMode = newMode;
      DataLogManager.log(getSubsystem( ) + ": move " + m_elbowMode + ((outOfRange) ? " - OUT OF RANGE" : ""));
    }

    m_elbowTargetDegrees = m_elbowCurDegrees;

    if (m_elbowValid)
      m_elbow.set(ControlMode.PercentOutput, axisValue * ELConsts.kElbowSpeedMaxManual);
  }

  public void setElbowStopped( )
  {
    DataLogManager.log(getSubsystem( ) + ": now STOPPED");

    if (m_elbowValid)
      m_elbow.set(ControlMode.PercentOutput, 0.0);
  }

  public void setElbowAngleToZero( )
  {
    m_elbow.setSelectedSensorPosition(elbowDegreesToCounts(0));
  }

  private double calculateTotalFF( )
  {
    double extensionLength = RobotContainer.getInstance( ).m_extension.getInches( );

    return Math.sin(Math.toRadians(m_elbowCurDegrees))
        * (ELConsts.kArbitraryFF + ELConsts.kExtArbFF * ((extensionLength) / EXConsts.kLengthMax));
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveElbowAngleInit(ElbowAngle angle)
  {
    if (angle != m_elbowAngle)
    {
      m_elbowAngle = angle;
      m_moveIsFinished = false;
      DataLogManager.log(String.format("%s: new mode request - %s", getSubsystem( ), m_elbowAngle));

      switch (m_elbowAngle)
      {
        default : // Fall through to NOCHANGE if invalid
          DataLogManager.log(String.format("%s: requested angle is invalid - %s", getSubsystem( ), m_elbowAngle));
        case ELBOW_NOCHANGE : // Do not change from current level!
          m_elbowTargetDegrees = m_elbowCurDegrees;
          if (m_elbowTargetDegrees < 0.25)
            m_elbowTargetDegrees = 0.25;
          break;
        case ELBOW_STOW :
          m_elbowTargetDegrees = ELConsts.kAngleStow;
          break;
        case ELBOW_IDLE :
          m_elbowTargetDegrees = ELConsts.kAngleScoreLow;
          break;
        case ELBOW_LOW :
          m_elbowTargetDegrees = ELConsts.kAngleScoreLow;
          break;
        case ELBOW_MID :
          m_elbowTargetDegrees = ELConsts.kAngleScoreMid;
          break;
        case ELBOW_HIGH :
          m_elbowTargetDegrees = ELConsts.kAngleScoreHigh;
          break;
        case ELBOW_SHELF :
          m_elbowTargetDegrees = ELConsts.kAngleSubstation;
          break;
      }
    }

    if (ELConsts.kCalibrated && moveIsInRange(Math.abs(m_elbowTargetDegrees - m_elbowCurDegrees)))
    {
      // angle constraint check/soft limit for max and min angle before raising
      if (!moveIsInRange(m_elbowTargetDegrees))
      {
        DataLogManager.log(String.format("%s: Target %.1f degrees is OUT OF RANGE! [%.1f, %.1f]", getSubsystem( ),
            m_elbowTargetDegrees, ELConsts.kAngleMin, ELConsts.kAngleMax));
        m_elbowTargetDegrees = m_elbowCurDegrees;
      }

      m_safetyTimer.restart( );

      if (m_elbowValid && ELConsts.kCalibrated)
        m_elbow.set(ControlMode.MotionMagic, elbowDegreesToCounts(m_elbowTargetDegrees), DemandType.ArbitraryFeedForward,
            m_elbowTotalFF);

      DataLogManager
          .log(String.format("%s: moving: %.1f -> %.1f degrees", getSubsystem( ), m_elbowCurDegrees, m_elbowTargetDegrees));
    }
    else
    {
      DataLogManager.log(getSubsystem( ) + ": not calibrated");
      if (m_elbowValid)
        m_elbow.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public void moveElbowAngleExecute( )
  {
    if (m_elbowValid && ELConsts.kCalibrated)
      m_elbow.set(ControlMode.MotionMagic, elbowDegreesToCounts(m_elbowTargetDegrees), DemandType.ArbitraryFeedForward,
          m_elbowTotalFF);
  }

  public boolean moveElbowAngleIsFinished( )
  {
    double errorInDegrees = 0.0;

    errorInDegrees = m_elbowTargetDegrees - m_elbowCurDegrees;

    if (Math.abs(errorInDegrees) < ELConsts.kToleranceDegrees)
    {
      if (++m_withinTolerance >= 3)
      {
        m_moveIsFinished = true;
        DataLogManager.log(String.format("%s: move finished - Time: %.3f  |  Cur degrees: %.1f", getSubsystem( ),
            m_safetyTimer.get( ), m_elbowCurDegrees));
      }
    }
    else
    {
      m_withinTolerance = 0;
    }

    if (m_safetyTimer.hasElapsed(ELConsts.kMMSafetyTimeout))
    {
      m_moveIsFinished = true;
      DataLogManager.log(getSubsystem( ) + ": Move Safety timer has timed out! " + m_safetyTimer.get( ));
    }

    if (m_moveIsFinished)
    {
      m_withinTolerance = 0;
      m_safetyTimer.stop( );
    }

    return (m_elbowAngle == ElbowAngle.ELBOW_NOCHANGE) ? false : m_moveIsFinished;
  }

  public boolean isElbowBelowIdle( )
  {
    int curCounts = (int) m_elbow.getSelectedSensorPosition(0);
    m_elbowCurDegrees = elbowCountsToDegrees(curCounts);
    return m_elbowCurDegrees < ELConsts.kAngleIdle;
  }

  public boolean isElbowBelowLow( )
  {
    int curCounts = (int) m_elbow.getSelectedSensorPosition(0);
    m_elbowCurDegrees = elbowCountsToDegrees(curCounts);
    return m_elbowCurDegrees < ELConsts.kAngleScoreLow;
  }

  public boolean isElbowBelowMid( )
  {
    int curCounts = (int) m_elbow.getSelectedSensorPosition(0);
    m_elbowCurDegrees = elbowCountsToDegrees(curCounts);
    return m_elbowCurDegrees < ELConsts.kAngleScoreMid;
  }

  public void moveElbowAngleEnd( )
  {
    m_moveIsFinished = false;
    m_withinTolerance = 0;
    m_safetyTimer.stop( );
    m_elbow.set(0.0);
  }

}
