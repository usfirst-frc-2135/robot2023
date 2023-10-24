//
// Arm subystem - wrist joint
//
package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
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
import frc.robot.Constants.Ports;
import frc.robot.Constants.WRConsts;
import frc.robot.Constants.WRConsts.WristMode;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.CTREConfigs6;
import frc.robot.lib.util.PhoenixUtil6;

//
// Wrist subsystem class
//
public class Wrist extends SubsystemBase
{
  // Constants
  private final double              kLigament2dOffset = 0.0; // Offset from mechanism root for wrist ligament

  // Member objects
  private final TalonFX             m_motor           = new TalonFX(Ports.kCANID_Wrist);
  private final CANcoder            m_CANCoder        = new CANcoder(Ports.kCANID_WRCANCoder);
  private final TalonFXSimState     m_motorSim        = m_motor.getSimState( );
  private final CANcoderSimState    m_CANCoderSim     = m_CANCoder.getSimState( );
  private final SingleJointedArmSim m_armSim          = new SingleJointedArmSim(DCMotor.getFalcon500(1), WRConsts.kGearRatio, 1.0,
      WRConsts.kGripperLengthMeters, -Math.PI, Math.PI, false);

  // Mechanism2d
  private final Mechanism2d         m_mech            = new Mechanism2d(3, 3);
  private final MechanismRoot2d     m_mechRoot        = m_mech.getRoot("wrist", 1.5, 2);
  private final MechanismLigament2d m_mechLigament    =
      m_mechRoot.append(new MechanismLigament2d("wrist", 0.5, kLigament2dOffset, 6, new Color8Bit(Color.kPurple)));

  // Declare module variables
  private boolean                   m_motorValid;      // Health indicator for Falcon 
  private boolean                   m_ccValid;         // Health indicator for CANCoder 
  private boolean                   m_calibrated      = true;
  private boolean                   m_debug           = true;

  private WristMode                 m_mode            = WristMode.WRIST_INIT;     // Manual movement mode with joysticks

  private double                    m_currentDegrees  = 0.0; // Current angle in degrees
  private double                    m_targetDegrees   = 0.0; // Target angle in degrees
  private Debouncer                 m_withinTolerance = new Debouncer(0.060, DebounceType.kRising);
  private boolean                   m_moveIsFinished;        // Movement has completed (within tolerance)

  private VoltageOut                m_requestVolts    = new VoltageOut(0).withEnableFOC(false);
  private MotionMagicVoltage        m_requestMMVolts  = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
  private double                    m_totalArbFeedForward;   // Arbitrary feedforward added to counteract gravity

  private Timer                     m_safetyTimer     = new Timer( ); // Safety timer for movements
  private StatusSignal<Double>      m_motorPosition   = m_motor.getRotorPosition( );
  private StatusSignal<Double>      m_motorVelocity   = m_motor.getRotorVelocity( );
  private StatusSignal<Double>      m_motorCLoopError = m_motor.getClosedLoopError( );
  private StatusSignal<Double>      m_motorSupplyCur  = m_motor.getSupplyCurrent( );
  private StatusSignal<Double>      m_motorStatorCur  = m_motor.getStatorCurrent( );
  private StatusSignal<Double>      m_ccPosition      = m_CANCoder.getAbsolutePosition( );

  // Constructor
  public Wrist( )
  {
    setName("Wrist");
    setSubsystem("Wrist");

    m_motorValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_motor, "wrist", CTREConfigs6.wristAngleFXConfig( ));
    m_ccValid = PhoenixUtil6.getInstance( ).canCoderInitialize6(m_CANCoder, "wrist", CTREConfigs6.wristCancoderConfig( ));

    if (Robot.isReal( ))
      m_currentDegrees = getCANCoderDegrees( );
    m_motor.setRotorPosition(Conversions.degreesToInputRotations(m_currentDegrees, WRConsts.kGearRatio));
    DataLogManager.log(String.format("%s: CANCoder initial degrees %.1f", getSubsystem( ), m_currentDegrees));

    m_motorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    m_CANCoderSim.Orientation = ChassisReference.Clockwise_Positive;

    m_motorPosition.setUpdateFrequency(50);
    if (m_debug)
    {
      m_motorVelocity.setUpdateFrequency(50);
      m_motorCLoopError.setUpdateFrequency(50);
      m_motorSupplyCur.setUpdateFrequency(50);
      m_motorStatorCur.setUpdateFrequency(50);
    }
    m_ccPosition.setUpdateFrequency(50);

    initSmartDashboard( );
    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    m_currentDegrees = getTalonFXDegrees( );
    m_totalArbFeedForward = calculateTotalArbFF( );
    SmartDashboard.putNumber("WR_curDegrees", m_currentDegrees);
    SmartDashboard.putNumber("WR_targetDegrees", m_targetDegrees);
    SmartDashboard.putNumber("WR_CCDegrees", getCANCoderDegrees( ));
    SmartDashboard.putNumber("WR_totalFF", m_totalArbFeedForward);
    if (m_debug)
    {
      SmartDashboard.putNumber("WR_velocity", m_motorVelocity.refresh( ).getValue( ));
      SmartDashboard.putNumber("WR_curError", m_motorCLoopError.refresh( ).getValue( ));
      SmartDashboard.putNumber("WR_supplyCur", m_motorSupplyCur.refresh( ).getValue( ));
      SmartDashboard.putNumber("WR_statorCur", m_motorStatorCur.refresh( ).getValue( ));
    }
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
    m_motorSim.setRawRotorPosition(Conversions.radiansToInputRotations(m_armSim.getAngleRads( ), WRConsts.kGearRatio));
    m_motorSim.setRotorVelocity(Conversions.radiansToInputRotations(m_armSim.getVelocityRadPerSec( ), WRConsts.kGearRatio));

    m_CANCoderSim.setRawPosition(Units.radiansToRotations(m_armSim.getAngleRads( )));
    m_CANCoderSim.setVelocity(Units.radiansToRotations(m_armSim.getVelocityRadPerSec( )));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps( )));

    m_mechLigament.setAngle(kLigament2dOffset - m_currentDegrees);
  }

  private void initSmartDashboard( )
  {
    // Initialize dashboard widgets
    SmartDashboard.putBoolean("HL_validWR", m_motorValid);
    SmartDashboard.putBoolean("HL_validWRCC", m_ccValid);
    SmartDashboard.putData("WristMech", m_mech);
    SmartDashboard.putNumber("WR_ArbFF", 0.0);
  }

  public void initialize( )
  {
    setStopped( );

    m_currentDegrees = getTalonFXDegrees( );
    m_targetDegrees = m_currentDegrees;
    DataLogManager.log(String.format("%s: Subsystem initialized! Target Degrees: %.1f", getSubsystem( ), m_targetDegrees));
  }

  private double calculateTotalArbFF( )
  {
    double elbowDegrees = RobotContainer.getInstance( ).m_elbow.getAngle( );
    double wristDegrees = getAngle( );

    return WRConsts.kArbitraryFF * Math.cos(Math.toRadians(elbowDegrees - wristDegrees));
  }

  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////

  public double getAngle( )
  {
    return m_currentDegrees;
  }

  private double getCANCoderDegrees( )
  {
    return Conversions.rotationsToOutputDegrees(m_ccPosition.refresh( ).getValue( ), 1.0);
  }

  private double getTalonFXDegrees( )
  {
    return Conversions.rotationsToOutputDegrees(m_motorPosition.refresh( ).getValue( ), WRConsts.kGearRatio);
  }

  public boolean isBelowIdle( )
  {
    return m_currentDegrees < WRConsts.kAngleIdle;
  }

  public boolean isBelowLow( )
  {
    return m_currentDegrees < WRConsts.kAngleScoreLow;
  }

  public boolean isBelowMid( )
  {
    return m_currentDegrees < WRConsts.kAngleScoreMid;
  }

  private boolean isMoveValid(double degrees)
  {
    return (degrees > WRConsts.kAngleMin) && (degrees < WRConsts.kAngleMax);
  }

  private boolean isWithinTolerance(double targetDegrees)
  {
    return (Math.abs(targetDegrees - m_currentDegrees) < WRConsts.kToleranceDegrees);
  }

  public void resetPositionToZero( )
  {
    if (m_motorValid)
      m_motor.setRotorPosition(Conversions.degreesToInputRotations(0, WRConsts.kGearRatio));
  }

  public void setStopped( )
  {
    DataLogManager.log(String.format("%s: now STOPPED", getSubsystem( )));
    m_motor.setControl(m_requestVolts.withOutput(0.0));
  }

  ///////////////////////// MANUAL MOVEMENT ///////////////////////////////////

  public void moveWithJoystick(XboxController joystick)
  {
    double axisValue = -joystick.getRightY( );
    boolean rangeLimited = false;
    WristMode newMode = WristMode.WRIST_STOPPED;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if (axisValue < 0.0)
    {
      if (m_currentDegrees > WRConsts.kAngleMin)
        newMode = WRConsts.WristMode.WRIST_UP;
      else
        rangeLimited = true;
    }
    else if (axisValue > 0.0)
    {
      if (m_currentDegrees < WRConsts.kAngleMax)
        newMode = WRConsts.WristMode.WRIST_DOWN;
      else
        rangeLimited = true;
    }

    if (rangeLimited)
      axisValue = 0.0;

    if (newMode != m_mode)
    {
      m_mode = newMode;
      DataLogManager.log(String.format("%s: move %s %.1f deg %s", getSubsystem( ), m_mode, getAngle( ),
          ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_targetDegrees = m_currentDegrees;

    m_motor.setControl(
        m_requestVolts.withOutput(axisValue * WRConsts.kManualSpeedVolts + SmartDashboard.getNumber("WR_ArbFF", 0.0))); // TODO: sine/cosine 
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveToPositionInit(double newAngle, boolean holdPosition)
  {
    m_safetyTimer.restart( );

    if (holdPosition)
      newAngle = getAngle( );

    // Decide if a new position request
    if (holdPosition || newAngle != m_targetDegrees || !isWithinTolerance(newAngle))
    {
      // Validate the position request
      if (isMoveValid(newAngle))
      {
        m_targetDegrees = newAngle;
        m_moveIsFinished = false;
        m_withinTolerance.calculate(false); // Reset the debounce filter

        m_motor
            .setControl(m_requestMMVolts.withPosition(Conversions.degreesToInputRotations(m_targetDegrees, WRConsts.kGearRatio))
                .withFeedForward(m_totalArbFeedForward));
        DataLogManager.log(String.format("%s: Position move: %.1f -> %.1f degrees (%.1f -> %.1f)", getSubsystem( ),
            m_currentDegrees, m_targetDegrees, Conversions.degreesToInputRotations(m_currentDegrees, WRConsts.kGearRatio),
            Conversions.degreesToInputRotations(m_targetDegrees, WRConsts.kGearRatio)));
      }
      else
        DataLogManager.log(String.format("%s: Position move %.1f degrees is OUT OF RANGE! [%.1f, %.1f]", getSubsystem( ),
            m_targetDegrees, WRConsts.kAngleMin, WRConsts.kAngleMax));
    }
    else
    {
      m_moveIsFinished = true;
      DataLogManager.log(String.format("%s: Position already achieved - %s", getSubsystem( ), m_targetDegrees));
    }
  }

  public void moveToPositionExecute( )
  {
    if (m_calibrated)
      m_motor.setControl(m_requestMMVolts.withPosition(Conversions.degreesToInputRotations(m_targetDegrees, WRConsts.kGearRatio))
          .withFeedForward(m_totalArbFeedForward));
  }

  public boolean moveToPositionIsFinished( )
  {
    boolean timedOut = m_safetyTimer.hasElapsed(WRConsts.kMMSafetyTimeout);
    double error = m_targetDegrees - m_currentDegrees;

    if (m_withinTolerance.calculate(Math.abs(error) < WRConsts.kToleranceDegrees) || timedOut)
    {
      if (!m_moveIsFinished)
        DataLogManager.log(String.format("%s: Position move finished - Current degrees: %.1f (error %.1f) - Time: %.3f %s",
            getSubsystem( ), m_currentDegrees, error, m_safetyTimer.get( ), (timedOut) ? "- TIMED OUT!" : ""));

      m_moveIsFinished = true;
    }

    return m_moveIsFinished;
  }

  public void moveToPositionEnd( )
  {
    m_safetyTimer.stop( );
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  // Manual commands for scoring cones in slam dunk fashion

  public void moveConstantSpeed(double speedInvolts)
  {
    boolean rangeLimited = false;
    WristMode newMode = WristMode.WRIST_STOPPED;

    speedInvolts = MathUtil.applyDeadband(speedInvolts, Constants.kStickDeadband);

    if (speedInvolts < 0.0)
    {
      if (m_currentDegrees > WRConsts.kAngleMin)
        newMode = WristMode.WRIST_UP;
      else
        rangeLimited = true;
    }
    else if (speedInvolts > 0.0)
    {
      if (m_currentDegrees < WRConsts.kAngleMax)
        newMode = WristMode.WRIST_DOWN;
      else
        rangeLimited = true;
    }

    if (rangeLimited)
      speedInvolts = 0.0;

    if (newMode != m_mode)
    {
      m_mode = newMode;
      DataLogManager.log(String.format("%s: move %s %.1f deg %s", getSubsystem( ), m_mode, getAngle( ),
          ((rangeLimited) ? " - OUT OF RANGE" : "")));
    }

    m_targetDegrees = m_currentDegrees;

    if (m_motorValid)
      m_motor.setControl(m_requestVolts.withOutput(speedInvolts));
  }

}
