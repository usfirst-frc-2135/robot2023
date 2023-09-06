//
// Arm subystem - elbow joint
//
package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
import frc.robot.Constants.ELConsts.ElbowMode;
import frc.robot.Constants.ELConsts.ElbowPosition;
import frc.robot.Constants.EXConsts;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.Phoenix6_CTREConfigs;
import frc.robot.team2135.PhoenixUtil6;

//
// Elbow subsystem class
//
public class Elbow extends SubsystemBase
{
  // Constants
  private final double              kLigament2dOffset = -90.0; // Offset from mechanism root for elbow ligament

  // Member objects
  private final TalonFX             m_motor           = new TalonFX(Constants.Ports.kCANID_Elbow);
  private final CANcoder            m_CANCoder        = new CANcoder(Constants.Ports.kCANID_ELCANCoder);
  private final TalonFXSimState     m_motorSim        = m_motor.getSimState( );
  private final CANcoderSimState    m_CANCoderSim     = m_CANCoder.getSimState( );
  private final SingleJointedArmSim m_armSim          = new SingleJointedArmSim(DCMotor.getFalcon500(1), ELConsts.kGearRatio, 1.0,
      ELConsts.kForearmLengthMeters, -Math.PI, Math.PI, false);

  // Mechanism2d
  private final Mechanism2d         m_mech            = new Mechanism2d(3, 3);
  private final MechanismRoot2d     m_mechRoot        = m_mech.getRoot("elbow", 1.5, 2);
  private final MechanismLigament2d m_mechLigament    =
      m_mechRoot.append(new MechanismLigament2d("elbow", 1, kLigament2dOffset, 6, new Color8Bit(Color.kBlue)));

  // Declare module variables
  private boolean                   m_motorValid;      // Health indicator for Falcon 
  private boolean                   m_ccValid;         // Health indicator for CANCoder 

  private ElbowMode                 m_mode            = ElbowMode.ELBOW_INIT;     // Manual movement mode with joysticks
  private ElbowPosition             m_position        = ElbowPosition.ELBOW_STOW; // Desired position (stow, idle, low, mid, high, etc.)

  private double                    m_currentDegrees  = 0.0; // Current angle in degrees
  private double                    m_targetDegrees   = 0.0; // Target angle in degrees
  private Debouncer                 m_withinTolerance = new Debouncer(0.60, DebounceType.kRising);
  private boolean                   m_moveIsFinished;        // Movement has completed (within tolerance)

  private VoltageOut                m_requestVolts    = new VoltageOut(0).withEnableFOC(false);
  private MotionMagicVoltage        m_requestMMVolts  = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
  private double                    m_totalArbFeedForward;   // Arbitrary feedforward added to counteract gravity

  private Timer                     m_safetyTimer     = new Timer( ); // Safety timer for movements

  // Constructor
  public Elbow( )
  {
    setName("Elbow");
    setSubsystem("Elbow");

    m_motorValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_motor, "elbow", Phoenix6_CTREConfigs.elbowMotorFXConfig( ));
    m_ccValid = PhoenixUtil6.getInstance( ).canCoderInitialize6(m_CANCoder, "elbow", Phoenix6_CTREConfigs.elbowCancoderConfig( ));

    if (Robot.isReal( ))
      m_currentDegrees = getCANCoderDegrees( );
    m_motor.setRotorPosition(Conversions.degreesToRotations(m_currentDegrees, ELConsts.kGearRatio));
    DataLogManager.log(String.format("%s: CANCoder initial degrees %.1f", getSubsystem( ), m_currentDegrees));

    m_motorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    m_CANCoderSim.Orientation = ChassisReference.Clockwise_Positive;

    initSmartDashboard( );
    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    m_currentDegrees = getTalonFXDegrees( );
    m_mechLigament.setAngle(m_currentDegrees + kLigament2dOffset);
    SmartDashboard.putNumber("EL_curDegrees", m_currentDegrees);
    SmartDashboard.putNumber("EL_targetDegrees", m_targetDegrees);
    SmartDashboard.putNumber("EL_CCDegrees", getCANCoderDegrees( ));

    m_totalArbFeedForward = calculateTotalArbFF( );
    SmartDashboard.putNumber("EL_totalFF", m_totalArbFeedForward);

    m_motor.getStatorCurrent( ).refresh( );
    SmartDashboard.putNumber("EL_currentDraw", m_motor.getStatorCurrent( ).getValue( ));
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input motor voltage from the motor setting
    m_motorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_CANCoderSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_armSim.setInput(m_motorSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_motorSim.setRawRotorPosition(
        Conversions.degreesToRotations(Units.radiansToDegrees(m_armSim.getAngleRads( )), ELConsts.kGearRatio));
    m_motorSim.setRotorVelocity(
        Conversions.degreesToRotations(Units.radiansToDegrees(m_armSim.getVelocityRadPerSec( )), ELConsts.kGearRatio));

    m_CANCoderSim.setRawPosition(Units.radiansToRotations(m_armSim.getAngleRads( )));
    m_CANCoderSim.setVelocity(Units.radiansToRotations(m_armSim.getVelocityRadPerSec( )));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps( )));
  }

  private void initSmartDashboard( )
  {
    // Initialize dashboard widgets
    SmartDashboard.putBoolean("HL_validEL", m_motorValid);
    SmartDashboard.putBoolean("HL_validELCC", m_ccValid);
    SmartDashboard.putData("ElbowMech", m_mech);
  }

  public void initialize( )
  {
    setElbowStopped( );

    m_currentDegrees = getTalonFXDegrees( );
    m_targetDegrees = m_currentDegrees;
    DataLogManager.log(String.format("%s: Subsystem initialized! Target Degrees: %.1f", getSubsystem( ), m_targetDegrees));
  }

  private double calculateTotalArbFF( )
  {
    double extensionLength = RobotContainer.getInstance( ).m_extension.getInches( );

    return Math.sin(Math.toRadians(m_currentDegrees))
        * (ELConsts.kArbitraryFF + ELConsts.kExtArbFF * ((extensionLength) / EXConsts.kLengthMax));
  }

  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////

  public double getAngle( )
  {
    return m_currentDegrees;
  }

  public double getCANCoderDegrees( )
  {
    return Conversions.rotationsToDegrees(m_CANCoder.getAbsolutePosition( ).refresh( ).getValue( ), 1.0);
  }

  public double getTalonFXDegrees( )
  {
    return Conversions.rotationsToDegrees(m_motor.getRotorPosition( ).refresh( ).getValue( ), ELConsts.kGearRatio);
  }

  public boolean isElbowBelowIdle( )
  {
    return m_currentDegrees < ELConsts.kAngleIdle;
  }

  public boolean isElbowBelowLow( )
  {
    return m_currentDegrees < ELConsts.kAngleScoreLow;
  }

  public boolean isElbowBelowMid( )
  {
    return m_currentDegrees < ELConsts.kAngleScoreMid;
  }

  public boolean isMoveValid(double degrees)
  {
    return (degrees > ELConsts.kAngleMin) && (degrees < ELConsts.kAngleMax);
  }

  private boolean isWithinTolerance(double targetDegrees)
  {
    return (Math.abs(targetDegrees - m_currentDegrees) < ELConsts.kToleranceDegrees);
  }

  public void setElbowAngleToZero( )
  {
    m_motor.setRotorPosition(Conversions.degreesToRotations(0, ELConsts.kGearRatio));
  }

  public void setElbowStopped( )
  {
    DataLogManager.log(String.format("%s: now STOPPED", getSubsystem( )));
    m_motor.setControl(m_requestVolts.withOutput(0.0));
  }

  ///////////////////////// MANUAL MOVEMENT ///////////////////////////////////

  public void moveElbowWithJoystick(XboxController joystick)
  {
    double axisValue = -joystick.getLeftY( );
    boolean rangeLimited = false;
    ElbowMode newMode = ElbowMode.ELBOW_STOPPED;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if (axisValue < 0.0)
    {
      if (m_currentDegrees > ELConsts.kAngleMin)
        newMode = ElbowMode.ELBOW_DOWN;
      else
        rangeLimited = true;
    }
    else if (axisValue > 0.0)
    {
      if (m_currentDegrees < ELConsts.kAngleMax)
        newMode = ElbowMode.ELBOW_UP;
      else
        rangeLimited = true;
    }

    if (rangeLimited)
      axisValue = 0.0;

    if (newMode != m_mode)
    {
      m_mode = newMode;
      DataLogManager.log(String.format("%s: move %s %s", getSubsystem( ), m_mode, ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_targetDegrees = m_currentDegrees;

    m_motor.setControl(m_requestVolts.withOutput(axisValue * ELConsts.kManualSpeedVolts));
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveElbowToPositionInit(ElbowPosition position)
  {
    double targetDegrees = 0.0;

    // Decode the position request
    switch (position)
    {
      default : // Fall through to NOCHANGE if invalid
        DataLogManager.log(String.format("%s: Position move request is invalid - %s", getSubsystem( ), position));
      case ELBOW_NOCHANGE : // Do not change from current level!
        targetDegrees = m_currentDegrees;
        break;
      case ELBOW_STOW :
        targetDegrees = ELConsts.kAngleStow;
        break;
      case ELBOW_IDLE :
        targetDegrees = ELConsts.kAngleScoreLow;
        break;
      case ELBOW_LOW :
        targetDegrees = ELConsts.kAngleScoreLow;
        break;
      case ELBOW_MID :
        targetDegrees = ELConsts.kAngleScoreMid;
        break;
      case ELBOW_HIGH :
        targetDegrees = ELConsts.kAngleScoreHigh;
        break;
      case ELBOW_SHELF :
        targetDegrees = ELConsts.kAngleSubstation;
        break;
    }

    m_safetyTimer.restart( );

    // Decide if a new position request
    if (position != m_position || !isWithinTolerance(targetDegrees))
    {
      // Validate the position request
      if (isMoveValid(targetDegrees))
      {
        m_position = position;
        m_targetDegrees = targetDegrees;
        m_moveIsFinished = false;
        m_withinTolerance.calculate(false); // Reset the debounce filter

        m_motor.setControl(m_requestMMVolts.withPosition(Conversions.degreesToRotations(m_targetDegrees, ELConsts.kGearRatio)));
        // .withFeedForward((m_totalArbFeedForward))); // TODO
        DataLogManager.log(String.format("%s: Position move %s: %.1f -> %.1f degrees (%.1f -> %.1f)", getSubsystem( ), m_position,
            m_currentDegrees, m_targetDegrees, Conversions.degreesToRotations(m_currentDegrees, ELConsts.kGearRatio),
            Conversions.degreesToRotations(m_targetDegrees, ELConsts.kGearRatio)));
      }
      else
        DataLogManager.log(String.format("%s: Position move %.1f degrees is OUT OF RANGE! [%.1f, %.1f]", getSubsystem( ),
            m_targetDegrees, ELConsts.kAngleMin, ELConsts.kAngleMax));
    }
    else
    {
      m_moveIsFinished = true;
      DataLogManager.log(String.format("%s: Position already achieved - %s", getSubsystem( ), m_position));
    }
  }

  public void moveElbowToPositionExecute( )
  {
    if (ELConsts.kCalibrated)
      m_motor.setControl(m_requestMMVolts.withPosition(Conversions.degreesToRotations(m_targetDegrees, ELConsts.kGearRatio)));
    // .withFeedForward(m_totalArbFeedForward)); // TODO
  }

  public boolean moveElbowToPositionIsFinished( )
  {

    if (m_withinTolerance.calculate((Math.abs(m_targetDegrees - m_currentDegrees) < ELConsts.kToleranceDegrees)))
    {
      m_moveIsFinished = true;
      DataLogManager.log(String.format("%s: Position move finished - Time: %.3f  |  Cur degrees: %.1f", getSubsystem( ),
          m_safetyTimer.get( ), m_currentDegrees));
    }

    if (m_safetyTimer.hasElapsed(ELConsts.kMMSafetyTimeout))
    {
      m_moveIsFinished = true;
      DataLogManager.log(String.format("%s: Position move safety timer timed out! %.1f", getSubsystem( ), m_safetyTimer.get( )));
    }

    return (m_position == ElbowPosition.ELBOW_NOCHANGE) ? false : m_moveIsFinished;
  }

  public void moveElbowToPositionEnd( )
  {
    m_safetyTimer.stop( );
    // m_motor.setControl(m_requestVolts.withOutput(0.0)); // TODO: Is this needed
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

}
