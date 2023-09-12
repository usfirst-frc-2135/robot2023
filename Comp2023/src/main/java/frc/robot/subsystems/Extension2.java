//
// Arm subystem - extension joint
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
import frc.robot.Constants.EXConsts.ExtensionMode;
import frc.robot.Constants.EXConsts;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.CTREConfigs6;
import frc.robot.team2135.PhoenixUtil6;

//
// Extension subsystem class
//
public class Extension2 extends SubsystemBase
{
  // Constants
  private final double              kLigament2dOffset = -90.0; // Offset from mechanism root for extension ligament

  // Member objects
  private final TalonFX             m_motor           = new TalonFX(Constants.Ports.kCANID_Extension);
  private final TalonFXSimState     m_motorSim        = m_motor.getSimState( );
  private final SingleJointedArmSim m_armSim          =
      new SingleJointedArmSim(DCMotor.getFalcon500(1), EXConsts.kGearRatio, 1.0, EXConsts.kLengthStow, -Math.PI, Math.PI, false);

  // Mechanism2d
  private final Mechanism2d         m_mech            = new Mechanism2d(3, 3);
  private final MechanismRoot2d     m_mechRoot        = m_mech.getRoot("extension", 1.5, 2);
  private final MechanismLigament2d m_mechLigament    =
      m_mechRoot.append(new MechanismLigament2d("extension", 1, kLigament2dOffset, 6, new Color8Bit(Color.kBlue)));

  // Declare module variables
  private boolean                   m_motorValid;      // Health indicator for Falcon 
  private boolean                   m_ccValid;         // Health indicator for CANCoder 

  private ExtensionMode             m_mode            = ExtensionMode.EXTENSION_INIT;     // Manual movement mode with joysticks

  private double                    m_currentInches   = 0.0; // Current length in inches
  private double                    m_targetInches    = 0.0; // Target length in inches
  private Debouncer                 m_withinTolerance = new Debouncer(0.060, DebounceType.kRising);
  private boolean                   m_moveIsFinished;        // Movement has completed (within tolerance)

  private VoltageOut                m_requestVolts    = new VoltageOut(0).withEnableFOC(false);
  private MotionMagicVoltage        m_requestMMVolts  = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
  private double                    m_totalArbFeedForward;   // Arbitrary feedforward added to counteract gravity

  private Timer                     m_safetyTimer     = new Timer( ); // Safety timer for movements

  // Constructor
  public Extension2( )
  {
    setName("Extension");
    setSubsystem("Extension");

    m_motorValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_motor, "extension", CTREConfigs6.extensionLengthFXConfig( ));

    if (Robot.isReal( ))
      m_currentInches = getCurrInches( );
    m_motor.setRotorPosition(inchesToOutputRotations(m_currentInches));
    DataLogManager.log(String.format("%s: CANCoder initial inches %.1f", getSubsystem( ), m_currentInches));

    m_motorSim.Orientation = ChassisReference.CounterClockwise_Positive;

    initSmartDashboard( );
    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    m_currentInches = getCurrInches( );
    m_mechLigament.setLength(m_currentInches + kLigament2dOffset);
    SmartDashboard.putNumber("EX_curInches", m_currentInches);
    SmartDashboard.putNumber("EX_targetInches", m_targetInches);

    m_totalArbFeedForward = calculateTotalArbFF( );
    SmartDashboard.putNumber("EX_totalFF", m_totalArbFeedForward);

    m_motor.getStatorCurrent( ).refresh( );
    SmartDashboard.putNumber("EX_currentDraw", m_motor.getStatorCurrent( ).getValue( ));
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input motor voltage from the motor setting
    m_motorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_armSim.setInput(m_motorSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_armSim.update(0.020);

    // // Finally, we set our simulated encoder's readings and simulated battery voltage TODO: Match for extension
    // m_motorSim.setRawRotorPosition(inchesToOutputRotations(m_armSim.( )));
    // m_motorSim.setRotorVelocity(Conversions.radiansToInputRotations(m_armSim.getVelocityRadPerSec( ), EXConsts.kGearRatio));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps( )));
  }

  private void initSmartDashboard( )
  {
    // Initialize dashboard widgets
    SmartDashboard.putBoolean("HL_validEX", m_motorValid);
    SmartDashboard.putBoolean("HL_validEXCC", m_ccValid);
    SmartDashboard.putData("ExtensionMech", m_mech);
  }

  public void initialize( )
  {
    setExtensionStopped( );

    m_currentInches = getCurrInches( );
    m_targetInches = m_currentInches;
    DataLogManager.log(String.format("%s: Subsystem initialized! Target Inches: %.1f", getSubsystem( ), m_targetInches));
  }

  private double calculateTotalArbFF( )
  {
    double elbowDegrees = RobotContainer.getInstance( ).m_elbow.getAngle( );

    return EXConsts.kArbitraryFF * Math.cos(Math.toRadians(elbowDegrees));
  }

  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////

  public double getCurrInches( )
  {
    return rotationsToOutputInches(m_motor.getRotorPosition( ).refresh( ).getValue( ));
  }

  public double rotationsToOutputInches(double rotations)
  {
    return rotations / EXConsts.kRolloutRatio;
  }

  public double inchesToOutputRotations(double inches)
  {
    return inches * EXConsts.kRolloutRatio;
  }

  public boolean isExtensionBelowIdle( )
  {
    return m_currentInches < EXConsts.kLengthIdle;
  }

  public boolean isExtensionBelowLow( )
  {
    return m_currentInches < EXConsts.kLengthScoreLow;
  }

  public boolean isExtensionBelowMid( )
  {
    return m_currentInches < EXConsts.kLengthScoreMid;
  }

  public boolean isMoveValid(double inches)
  {
    return (inches > EXConsts.kLengthMin) && (inches < EXConsts.kLengthMax);
  }

  private boolean isWithinTolerance(double targetInches)
  {
    return (Math.abs(targetInches - m_currentInches) < EXConsts.kToleranceInches);
  }

  public void setExtensionLengthToZero( )
  {
    m_motor.setRotorPosition(inchesToOutputRotations(0));
  }

  public void setExtensionStopped( )
  {
    DataLogManager.log(String.format("%s: now STOPPED", getSubsystem( )));
    m_motor.setControl(m_requestVolts.withOutput(0.0));
  }

  ///////////////////////// MANUAL MOVEMENT ///////////////////////////////////

  public void moveExtensionWithJoystick(XboxController joystick)
  {
    double axisValue = -joystick.getLeftY( );
    boolean rangeLimited = false;
    ExtensionMode newMode = ExtensionMode.EXTENSION_STOPPED;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if (axisValue < 0.0)
    {
      if (m_currentInches > EXConsts.kLengthMin)
        newMode = ExtensionMode.EXTENSION_OUT; //TODO: Check
      else
        rangeLimited = true;
    }
    else if (axisValue > 0.0)
    {
      if (m_currentInches < EXConsts.kLengthMax) //TODO: Check
        newMode = ExtensionMode.EXTENSION_IN;
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

    m_targetInches = m_currentInches;

    m_motor.setControl(m_requestVolts.withOutput(axisValue * EXConsts.kExtensionSpeedMaxManual)); // TODO: Check
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveExtensionToPositionInit(double newLength, boolean holdPosition)
  {
    m_safetyTimer.restart( );

    if (holdPosition)
      newLength = getCurrInches( );

    // Decide if a new position request
    if (holdPosition || newLength != m_targetInches || !isWithinTolerance(newLength))
    {
      // Validate the position request
      if (isMoveValid(newLength))
      {
        m_targetInches = newLength;
        m_moveIsFinished = false;
        m_withinTolerance.calculate(false); // Reset the debounce filter

        m_motor.setControl(m_requestMMVolts.withPosition(inchesToOutputRotations(m_targetInches)));
        // .withFeedForward((m_totalArbFeedForward)));  // TODO - once extension is fixed and Tuner X is used
        DataLogManager.log(String.format("%s: Position move: %.1f -> %.1f inches (%.1f -> %.1f)", getSubsystem( ),
            m_currentInches, m_targetInches, inchesToOutputRotations(m_currentInches), inchesToOutputRotations(m_targetInches)));
      }
      else
        DataLogManager.log(String.format("%s: Position move %.1f inches is OUT OF RANGE! [%.1f, %.1f]", getSubsystem( ),
            m_targetInches, EXConsts.kLengthMin, EXConsts.kLengthMax));

    }
    else
    {
      m_moveIsFinished = true;
      DataLogManager.log(String.format("%s: Position already achieved - %s", getSubsystem( ), m_targetInches));
    }
  }

  public void moveExtensionToPositionExecute( )
  {
    if (EXConsts.kCalibrated)
      m_motor.setControl(m_requestMMVolts.withPosition(inchesToOutputRotations(m_targetInches)));
    // .withFeedForward(m_totalArbFeedForward)); // TODO - once extension is fixed and Tuner X is used
  }

  public boolean moveExtensionToPositionIsFinished( )
  { //TODO: check
    boolean timedOut = m_safetyTimer.hasElapsed(EXConsts.kMMSafetyTimeoutRatio); //TODO: check
    double error = m_targetInches - m_currentInches;

    if (m_withinTolerance.calculate(Math.abs(error) < EXConsts.kToleranceInches) || timedOut)
    {
      if (!m_moveIsFinished)
        DataLogManager.log(String.format("%s: Position move finished - Current inches: %.1f (error %.1f) - Time: %.3f %s",
            getSubsystem( ), m_currentInches, error, m_safetyTimer.get( ), (timedOut) ? "- TIMED OUT!" : ""));

      m_moveIsFinished = true;
    }

    return m_moveIsFinished;
  }

  public void moveExtensionToPositionEnd( )
  {
    m_safetyTimer.stop( );
    // m_motor.setControl(m_requestVolts.withOutput(0.0)); // TODO: Is this needed? It fixed a bug in Motion Magic in v5 that should be fixed in v6
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

}
