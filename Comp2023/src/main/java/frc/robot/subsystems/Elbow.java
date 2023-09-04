//
// Arm subystem - elbow joint
//
package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
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
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.ELConsts.ElbowMode;
import frc.robot.Constants.ELConsts.ElbowPosition;
import frc.robot.Constants.EXConsts;
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

  // Member objects
  private final TalonFX             m_motor            = new TalonFX(Constants.Ports.kCANID_Elbow);  //elbow
  private final CANcoder            m_CANCoder         = new CANcoder(Constants.Ports.kCANID_ELCANCoder);
  private final TalonFXSimState     m_motorSim         = new TalonFXSimState(m_motor);
  private final CANcoderSimState    m_CANCoderSim      = new CANcoderSimState(m_CANCoder);
  private final SingleJointedArmSim m_elbowSim         = new SingleJointedArmSim(DCMotor.getFalcon500(1), ELConsts.kGearRatio,
      2.0, ELConsts.kForearmLengthMeters, -Math.PI, Math.PI, false);

  // Mechanism2d
  private final Mechanism2d         m_mech             = new Mechanism2d(3, 3);
  private final MechanismRoot2d     m_mechRoot         = m_mech.getRoot("elbow", 1.5, 2);
  private final MechanismLigament2d m_mechLigament     =
      m_mechRoot.append(new MechanismLigament2d("elbow", 1, 0, 6, new Color8Bit(Color.kBlue)));

  // Declare module variables
  private boolean                   m_motorValid;      // Health indicator for Falcon 
  private boolean                   m_ccValid;         // Health indicator for CANCoder 

  private ElbowMode                 m_mode             = ElbowMode.ELBOW_INIT;     // Manual state active with joysticks
  private ElbowPosition             m_position         = ElbowPosition.ELBOW_STOW; // Desired position (stow, idle, low, mid, high, etc.)

  private double                    m_currentDegrees   = 0.0; // Current angle in degrees
  private double                    m_targetDegrees    = 0.0; // Target angle in degrees
  private int                       m_inToleranceCount = 0;   // Counter for consecutive readings within tolerance
  private boolean                   m_moveIsFinished;         // Movement has completed (within tolerance)
  private double                    m_totalArbFeedForward;    // Arbitrary feedforward added to counteract gravity

  private VoltageOut                m_request          = new VoltageOut(0);           // Voltage mode control request
  private MotionMagicVoltage        m_mmRequest        = new MotionMagicVoltage(0); // Motion magic control request

  private Timer                     m_safetyTimer      = new Timer( ); // Safety timer for movements

  // Constructor
  public Elbow( )
  {
    setName("Elbow");
    setSubsystem("Elbow");

    m_motorValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_motor, "elbow", Phoenix6_CTREConfigs.elbowMotorFXConfig( ));
    m_ccValid = PhoenixUtil6.getInstance( ).canCoderInitialize6(m_CANCoder, "elbow", Phoenix6_CTREConfigs.elbowCancoderConfig( ));

    // TODO remove? Slow status frame updates AFTER getting the absolute position
    // m_CANCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
    // m_CANCoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);

    if (m_ccValid)  // Set TalonFX position from CANCoder absolute position
    {
      m_currentDegrees = getCanCoder( ).getDegrees( );
      DataLogManager.log(String.format("%s: CANCoder initial degrees %.1f", getSubsystem( ), m_currentDegrees));

      if (RobotBase.isReal( ))
        m_motor.setRotorPosition(Conversions.degreesToRotations(m_currentDegrees, ELConsts.kGearRatio));
    }

    initSmartDashboard( );
    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    m_currentDegrees = Conversions.rotationsToDegrees(m_motor.getRotorPosition( ).getValue( ), ELConsts.kGearRatio);

    if (m_motorValid)
    {
      SmartDashboard.putNumber("EL_curDegrees", m_currentDegrees);
      SmartDashboard.putNumber("EL_targetDegrees", m_targetDegrees);
      m_mechLigament.setAngle(m_currentDegrees);

      m_totalArbFeedForward = calculateTotalArbFF( );
      SmartDashboard.putNumber("EL_totalFF", m_totalArbFeedForward);
      SmartDashboard.putNumber("EL_currentDraw", m_motor.getStatorCurrent( ).getValue( ));
    }
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input motor voltage from the motor setting
    m_motorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_CANCoderSim.setSupplyVoltage(RobotController.getInputVoltage( ));

    m_elbowSim.setInput(m_motorSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_elbowSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_motorSim.setRawRotorPosition(Conversions.degreesToRotations(Units.radiansToDegrees(m_elbowSim.getAngleRads( )), ELConsts.kGearRatio));
    m_motorSim.setRotorVelocity(Conversions.degreesToRotations(Units.radiansToDegrees(m_elbowSim.getVelocityRadPerSec( )), ELConsts.kGearRatio));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elbowSim.getCurrentDrawAmps( )));
  }

  private void initSmartDashboard( )
  {
    // Initialize Variables
    SmartDashboard.putBoolean("HL_validEL", m_motorValid);
    SmartDashboard.putNumber("EL_curDegrees", m_currentDegrees);
    SmartDashboard.putNumber("EL_targetDegrees", m_targetDegrees);

    // post the mechanism to the dashboard
    SmartDashboard.putData("ElbowMech", m_mech);
  }

  public void initialize( )
  {
    double currentRotations = 0.0;

    setElbowStopped( );

    if (m_motorValid)
      currentRotations = m_motor.getRotorPosition( ).getValue( );
    m_currentDegrees = Conversions.rotationsToDegrees(currentRotations, ELConsts.kGearRatio);
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

  public Rotation2d getCanCoder( )
  {
    return Rotation2d.fromDegrees(m_CANCoder.getAbsolutePosition( ).getValue( ));
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

  public boolean isMoveInRange(double degrees)
  {
    return (degrees > ELConsts.kAngleMin) && (degrees < ELConsts.kAngleMax);
  }

  public void setElbowAngleToZero( )
  {
    m_motor.setRotorPosition(Conversions.degreesToRotations(0, ELConsts.kGearRatio));
  }

  public void setElbowStopped( )
  {
    DataLogManager.log(String.format("%s: now STOPPED", getSubsystem( )));

    if (m_motorValid)
      m_motor.setControl(m_request.withOutput(0.0));
  }

  ///////////////////////// MANUAL MOVEMENT ///////////////////////////////////

  public void moveElbowWithJoystick(XboxController joystick)
  {
    double axisValue = -joystick.getLeftY( );
    boolean outOfRange = false;
    ElbowMode newMode = ElbowMode.ELBOW_STOPPED;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if (axisValue < 0.0)
    {
      if (m_currentDegrees > ELConsts.kAngleMin)
        newMode = ElbowMode.ELBOW_DOWN;
      else
        outOfRange = true;
    }
    else if (axisValue > 0.0)
    {
      if (m_currentDegrees < ELConsts.kAngleMax)
        newMode = ElbowMode.ELBOW_UP;
      else
        outOfRange = true;
    }

    if (outOfRange)
      axisValue = 0.0;

    if (newMode != m_mode)
    {
      m_mode = newMode;
      DataLogManager.log(String.format("%s: move %s %s", getSubsystem( ), m_mode, ((outOfRange) ? " - OUT OF RANGE" : "")));
    }

    m_targetDegrees = m_currentDegrees;

    if (m_motorValid)
      m_motor.setControl(m_request.withOutput(axisValue * ELConsts.kManualSpeedMax));
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveElbowToPositionInit(ElbowPosition position)
  {
    if (position != m_position)
    {
      m_position = position;
      m_moveIsFinished = false;
      DataLogManager.log(String.format("%s: new mode request - %s", getSubsystem( ), m_position));

      switch (m_position)
      {
        default : // Fall through to NOCHANGE if invalid
          DataLogManager.log(String.format("%s: requested position is invalid - %s", getSubsystem( ), m_position));
        case ELBOW_NOCHANGE : // Do not change from current level!
          m_targetDegrees = m_currentDegrees;
          if (m_targetDegrees < 0.25)
            m_targetDegrees = 0.25;
          break;
        case ELBOW_STOW :
          m_targetDegrees = ELConsts.kAngleStow;
          break;
        case ELBOW_IDLE :
          m_targetDegrees = ELConsts.kAngleScoreLow;
          break;
        case ELBOW_LOW :
          m_targetDegrees = ELConsts.kAngleScoreLow;
          break;
        case ELBOW_MID :
          m_targetDegrees = ELConsts.kAngleScoreMid;
          break;
        case ELBOW_HIGH :
          m_targetDegrees = ELConsts.kAngleScoreHigh;
          break;
        case ELBOW_SHELF :
          m_targetDegrees = ELConsts.kAngleSubstation;
          break;
      }
    }

    if (ELConsts.kCalibrated && isMoveInRange(Math.abs(m_targetDegrees - m_currentDegrees)))
    {
      // angle constraint check/soft limit for max and min angle before raising
      if (!isMoveInRange(m_targetDegrees))
      {
        DataLogManager.log(String.format("%s: Target %.1f degrees is OUT OF RANGE! [%.1f, %.1f]", getSubsystem( ),
            m_targetDegrees, ELConsts.kAngleMin, ELConsts.kAngleMax));
        m_targetDegrees = m_currentDegrees;
      }

      m_safetyTimer.restart( );

      if (m_motorValid)
        m_motor.setControl(m_mmRequest.withPosition(Conversions.degreesToRotations(m_targetDegrees, ELConsts.kGearRatio))
            .withFeedForward((m_totalArbFeedForward)));
      DataLogManager.log(String.format("%s: moving: %.1f -> %.1f degrees", getSubsystem( ), m_currentDegrees, m_targetDegrees));
    }
    else
    {
      DataLogManager.log(String.format("%s: not calibrated or out of range!", getSubsystem( )));
      if (m_motorValid)
        m_motor.setControl(m_request.withOutput(0.0));
    }
  }

  public void moveElbowToPositionExecute( )
  {
    if (m_motorValid && ELConsts.kCalibrated)
      m_motor.setControl(m_mmRequest.withPosition(Conversions.degreesToRotations(m_targetDegrees, ELConsts.kGearRatio))
          .withFeedForward(m_totalArbFeedForward));
  }

  public boolean moveElbowToPositionIsFinished( )
  {
    double errorInDegrees = 0.0;

    errorInDegrees = m_targetDegrees - m_currentDegrees;

    if (Math.abs(errorInDegrees) < ELConsts.kToleranceDegrees)
    {
      if (++m_inToleranceCount >= 3)
      {
        m_moveIsFinished = true;
        DataLogManager.log(String.format("%s: move finished - Time: %.3f  |  Cur degrees: %.1f", getSubsystem( ),
            m_safetyTimer.get( ), m_currentDegrees));
      }
    }
    else
    {
      m_inToleranceCount = 0;
    }

    if (m_safetyTimer.hasElapsed(ELConsts.kMMSafetyTimeout))
    {
      m_moveIsFinished = true;
      DataLogManager.log(String.format("%s: Move Safety timer has timed out! %.1f", getSubsystem( ), m_safetyTimer.get( )));
    }

    if (m_moveIsFinished)
    {
      m_inToleranceCount = 0;
      m_safetyTimer.stop( );
    }

    return (m_position == ElbowPosition.ELBOW_NOCHANGE) ? false : m_moveIsFinished;
  }

  public void moveElbowToPositionEnd( )
  {
    m_moveIsFinished = false;
    m_inToleranceCount = 0;
    m_safetyTimer.stop( );
    m_motor.setControl(m_request.withOutput(0.0));
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

}
