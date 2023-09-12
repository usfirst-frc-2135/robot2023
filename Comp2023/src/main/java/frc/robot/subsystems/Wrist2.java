//
// Arm subystem - wrist joint
//
package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

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
public class Wrist2 extends SubsystemBase
{
  // Constants
  private static final int          PIDINDEX             = 0;   // PID in use (0-primary, 1-aux)
  private static final int          SLOTINDEX            = 0;   // Use first PID slot

  // Member objects
  private final TalonFX             m_wrist              = new TalonFX(Constants.Ports.kCANID_Wrist);  //wrist
  private final CANCoder            m_wristCANCoder      = new CANCoder(Constants.Ports.kCANID_WRCANCoder);
  private final TalonFXSimState     m_wristMotorSim      = m_wrist.getSimState( );
  private final SingleJointedArmSim m_wristSim           = new SingleJointedArmSim(DCMotor.getFalcon500(1), WRConsts.kGearRatio,
      2.0, WRConsts.kGripperLengthMeters, -Math.PI, Math.PI, false);

  // Mechanism2d
  private final Mechanism2d         m_wristMech          = new Mechanism2d(3, 3);
  private final MechanismRoot2d     m_wristRoot          = m_wristMech.getRoot("wrist", 1.5, 2);
  private final MechanismLigament2d m_wristLigament      =
      m_wristRoot.append(new MechanismLigament2d("wrist", 0.5, 0, 6, new Color8Bit(Color.kPurple)));

  private boolean                   m_wristValid;                 // Health indicator for wrist Talon 
  private boolean                   m_wristCCValid;               // Health indicator for wrist CANCoder 

  private double                    curWristRotations    = 0.0;

  private VoltageOut                m_requestVolts       = new VoltageOut(0).withEnableFOC(false);
  private MotionMagicVoltage        m_requestMMVolts     = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);

  // Declare module variables
  private WristMode                 m_wristMode          = WristMode.WRIST_INIT;              // Mode active with joysticks

  private WristAngle                m_wristAngle;                   // Desired extension length
  private boolean                   m_moveIsFinished;
  private double                    m_wristTargetDegrees = 0.0;    // Target angle in degrees
  private double                    m_wristCurDegrees    = 0.0;    // Current angle in degrees
  private int                       m_withinTolerance    = 0;      // Counter for consecutive readings within tolerance
  private double                    m_wristTotalFF;

  private Timer                     m_safetyTimer        = new Timer( ); // Safety timer for use in wrist

  // Constructor
  public Wrist2( )
  {
    setName("Wrist");
    setSubsystem("Wrist");

    m_wristValid = PhoenixUtil.getInstance( ).talonFXInitialize(m_wrist, "wrist");
    m_wristCCValid = PhoenixUtil.getInstance( ).canCoderInitialize(m_wristCANCoder, "wrist");

    if (m_wristCCValid)
    {
      m_wristCANCoder.configAllSettings(CTREConfigs.wristCancoderConfig( ));
      m_wristCurDegrees = getCanCoder( ).getDegrees( );

      //TODO:
      // // Slow status frame updates AFTER getting the absolute position
      // m_wristCANCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
      // m_wristCANCoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);

      // DataLogManager.log(getSubsystem( ) + ": Initial degrees " + m_wristCurDegrees);
      // double absolutePosition = Conversions.degreesToFalcon(m_wristCurDegrees, WRConsts.kGearRatio);

      if (RobotBase.isReal( ))
        m_wrist.setRotorPosition(0.0);
    }

    initSmartDashboard( );

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    if (m_wristValid)
    {
      curWristRotations = getCurrWristRotations( );

      m_wristCurDegrees = wristRotationsToDegrees((int) curWristRotations);
      SmartDashboard.putNumber("WR_curDegrees", m_wristCurDegrees);
      SmartDashboard.putNumber("WR_targetDegrees", m_wristTargetDegrees);
      m_wristLigament.setAngle(m_wristCurDegrees);

      m_wristTotalFF = calculateTotalFF( );
      SmartDashboard.putNumber("WR_totalFF", m_wristTotalFF);

      double currentDraw = m_wrist.getStatorCurrent( ).getValue( );
      SmartDashboard.putNumber("WR_currentDraw", currentDraw);
    }
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input motor voltage from the motor setting
    m_wristMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_wristSim.setInput(m_wristMotorSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_wristSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_wristMotorSim.setRawRotorPosition(wristDegreesToRotations(Units.radiansToDegrees(m_wristSim.getAngleRads( ))));
    m_wristMotorSim.setRotorVelocity(wristDegreesToRotations(Units.radiansToDegrees(m_wristSim.getVelocityRadPerSec( ))));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_wristSim.getCurrentDrawAmps( )));
  }

  private double wristDegreesToRotations(double radiansToDegrees)
  {
    return 0;
  }

  private void initSmartDashboard( )
  {
    SmartDashboard.putBoolean("HL_validWR", m_wristValid);

    // Initialize Variables
    SmartDashboard.putNumber("WR_curDegrees", m_wristCurDegrees);
    SmartDashboard.putNumber("WR_targetDegrees", m_wristTargetDegrees);
    SmartDashboard.putBoolean("WR_calibrated", WRConsts.kCalibrated);

    // post the mechanism to the dashboard
    SmartDashboard.putData("WristMech", m_wristMech);
  }

  public void initialize( )
  {
    double curWRRotations = 0.0;

    DataLogManager.log(getSubsystem( ) + ": Subsystem initialized!");

    setWristStopped( );

    if (m_wristValid)
      curWRRotations = getCurrWristRotations( );
    m_wristCurDegrees = wristRotationsToDegrees((int) curWRRotations);
    m_wristTargetDegrees = m_wristCurDegrees;
    DataLogManager.log(String.format("%s: Init Target Degrees: %.1f", getSubsystem( ), m_wristTargetDegrees));
  }

  public Rotation2d getCanCoder( )
  {
    return Rotation2d.fromDegrees(m_wristCANCoder.getAbsolutePosition( ));
  }

  private int wristDegreesToCounts(double degrees)
  {
    return (int) (degrees / WRConsts.kDegreesPerCount);
  }

  private double wristCountsToDegrees(int counts)
  {
    return counts * WRConsts.kDegreesPerCount;
  }

  private double wristRotationsToDegrees(int rotations)
  {
    return Conversions.rotationsToOutputDegrees(rotations, WRConsts.kGearRatio);
  }

  public boolean moveIsInRange(double degrees)
  {
    return (degrees > WRConsts.kAngleMin) && (degrees < WRConsts.kAngleMax);
  }

  public double getAngle( )
  {
    return m_wristCurDegrees;
  }

  // private void wristTalonInitialize(WPI_TalonFX motor, boolean inverted)
  // {
  //   //motor.configFactoryDefault( ); - TODO Clean up later
  //   motor.configAllSettings(CTREConfigs.wristAngleFXConfig( ));
  //   PhoenixUtil.getInstance( ).checkTalonError(motor, "configAllSettings");

  //   motor.setInverted(inverted);
  //   PhoenixUtil.getInstance( ).checkTalonError(motor, "setInverted");
  //   motor.setNeutralMode(NeutralMode.Brake);
  //   PhoenixUtil.getInstance( ).checkTalonError(motor, "setNeutralMode");
  //   motor.setSafetyEnabled(false);
  //   PhoenixUtil.getInstance( ).checkTalonError(motor, "setSafetyEnabled");
  //   motor.enableVoltageCompensation(true);
  //   PhoenixUtil.getInstance( ).checkTalonError(motor, "enableVoltageCompensation");

  //   // Configure sensor settings
  //   motor.setSelectedSensorPosition(0.0);
  //   PhoenixUtil.getInstance( ).checkTalonError(motor, "setSelectedSensorPosition");

  //   // Configure Magic Motion settings
  //   motor.selectProfileSlot(SLOTINDEX, PIDINDEX);
  //   PhoenixUtil.getInstance( ).checkTalonError(motor, "selectProfileSlot");

  //   motor.set(ControlMode.PercentOutput, 0.0);
  // }

  public void moveWristWithJoystick(XboxController joystick)
  {
    double axisValue = -joystick.getRightY( );
    moveWristInput(axisValue);
  }

  public void moveWristConstantSpeed(double speed)
  {
    moveWristInput(speed);
  }

  public void moveWristInput(double axisValue)
  {
    boolean outOfRange = false;
    WristMode newMode = WristMode.WRIST_STOPPED;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if (axisValue < 0.0)
    {
      if (m_wristCurDegrees > WRConsts.kAngleMin)
        newMode = WristMode.WRIST_UP;
      else
        outOfRange = true;
    }
    else if (axisValue > 0.0)
    {
      if (m_wristCurDegrees < WRConsts.kAngleMax)
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
      m_wrist.setControl(m_requestVolts.withOutput(axisValue));
  }

  public void setWristStopped( )
  {
    DataLogManager.log(getSubsystem( ) + ": now STOPPED");

    if (m_wristValid)
      m_wrist.setControl(m_requestVolts.withOutput(0.0));
  }

  //m_wrsit::methodYouWantToDO
  public void setWristAngleToZero( )
  {
    m_wrist.setRotorPosition(Conversions.degreesToInputRotations(0, WRConsts.kGearRatio));
  }

  private double calculateTotalFF( )
  {
    double elbowDegrees = RobotContainer.getInstance( ).m_elbow.getAngle( );
    double wristDegrees = RobotContainer.getInstance( ).m_wrist.getAngle( );

    return WRConsts.kArbitraryFF * Math.cos(Math.toRadians(elbowDegrees - wristDegrees));
  }

  public void setMotorOutput(double brake)
  {
    m_wrist.setControl(m_requestVolts.withOutput(brake));
  }

  public double getCurrWristRotations( )
  {
    return (double) (m_wrist.getRotorPosition( ).refresh( ).getValue( ));
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveWristAngleInit(WristAngle angle)
  {
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
          m_wristTargetDegrees = WRConsts.kAngleStow;
          break;
        case WRIST_IDLE :
          m_wristTargetDegrees = WRConsts.kAngleIdle;
          break;
        case WRIST_LOW :
          m_wristTargetDegrees = WRConsts.kAngleScoreLow;
          break;
        case WRIST_MID :
          m_wristTargetDegrees = WRConsts.kAngleScoreMid;
          break;
        case WRIST_HIGH :
          m_wristTargetDegrees = WRConsts.kAngleScoreHigh;
          break;
        case WRIST_SHELF :
          m_wristTargetDegrees = WRConsts.kAngleSubstation;
          break;
        case WRIST_SCORE :
          m_wristTargetDegrees = WRConsts.kAngleScore;
          break;
      }
    }

    if (WRConsts.kCalibrated && moveIsInRange(Math.abs(m_wristTargetDegrees - m_wristCurDegrees)))
    {
      // angle constraint check/soft limit for max and min angle before raising
      if (!moveIsInRange(m_wristTargetDegrees))
      {
        DataLogManager.log(String.format("%s: Target %.1f degrees is OUT OF RANGE! [%.1f, %.1f]", getSubsystem( ),
            m_wristTargetDegrees, WRConsts.kAngleMin, WRConsts.kAngleMax));
        m_wristTargetDegrees = m_wristCurDegrees;
      }

      m_safetyTimer.restart( );

      if (m_wristValid && WRConsts.kCalibrated)
        m_wrist.setControl(
            m_requestMMVolts.withPosition(Conversions.degreesToInputRotations(m_wristTargetDegrees, WRConsts.kGearRatio)));
      //, DemandType.ArbitraryFeedForward, m_wristTotalFF); // TODO: implement FF

      DataLogManager
          .log(String.format("%s: moving: %.1f -> %.1f degrees", getSubsystem( ), m_wristCurDegrees, m_wristTargetDegrees));
    }
    else
    {
      DataLogManager.log(getSubsystem( ) + ": not calibrated");
      if (m_wristValid)
        m_wrist.setControl(m_requestVolts.withOutput(0.0));
    }
  }

  public void moveWristAngleExecute( )
  {
    if (m_wristValid && WRConsts.kCalibrated)
      m_wrist.setControl(
          m_requestMMVolts.withPosition(Conversions.degreesToInputRotations(m_wristTargetDegrees, WRConsts.kGearRatio)));
    //, DemandType.ArbitraryFeedForward, m_wristTotalFF); // TODO: implement FF

  }

  public boolean moveWristAngleIsFinished( )
  {
    double errorInDegrees = 0.0;

    errorInDegrees = m_wristTargetDegrees - m_wristCurDegrees;

    if (Math.abs(errorInDegrees) < WRConsts.kToleranceDegrees)
    {
      if (++m_withinTolerance >= 3)
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
      DataLogManager.log(getSubsystem( ) + ": Move Safety timer has timed out! " + m_safetyTimer.get( ));
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
    m_wrist.set(0.0);
  }

}
