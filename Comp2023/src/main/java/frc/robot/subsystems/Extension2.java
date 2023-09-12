//
// Arm subystem - extension joint
//
package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
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
import frc.robot.RobotContainer;
import frc.robot.lib.util.CTREConfigs6;
import frc.robot.team2135.PhoenixUtil6;

//
// Extension subsystem class
//
public class Extension2 extends SubsystemBase
{
  // Constants
  private static final int      PIDINDEX                = 0;   // PID in use (0-primary, 1-aux)
  private static final int      SLOTINDEX               = 0;   // Use first PID slot

  // Member objects
  private final TalonFX         m_extension             = new TalonFX(Constants.Ports.kCANID_Extension);  //extension
  // private final TalonFXSimCollection      m_extensionMotorSim     = new TalonFXSimCollection(m_extension);
  // private final SingleJointedArmSim       m_extensionSim          = new SingleJointedArmSim(DCMotor.getFalcon500(1),
  //     EXConsts.kExtensionGearRatio, 2.0, EXConsts.kForearmLengthMeters, 0.0, Math.PI, false);

  // Mechanism2d
  private final Mechanism2d     m_extensionMech         = new Mechanism2d(3, 3);
  private final MechanismRoot2d m_extensionRoot         = m_extensionMech.getRoot("extension", 1.5, 2);
  private MechanismLigament2d   m_extensionLigament     =
      m_extensionRoot.append(new MechanismLigament2d("extension", 1, 0, 6, new Color8Bit(Color.kBlue)));

  private boolean               m_extensionValid;              // Health indicator for extension Talon 
  // private double                          m_extensionLengthOffset = 0.0; // CANCoder length measured at reference point

  // Declare module variables
  private ExtensionMode         m_extensionMode         = ExtensionMode.EXTENSION_INIT;          // Mode active with joysticks
  private VoltageOut            m_requestVolts          = new VoltageOut(0).withEnableFOC(false);
  private MotionMagicVoltage    m_requestMMVolts        = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
  private double                m_extensionCurRotations = 0.0;

  private ExtensionLength       m_extensionLength;                // Desired extension length
  private boolean               m_moveIsFinished;
  private double                m_extensionTargetInches = 0.0;    // Target length in inches
  private double                m_extensionCurInches    = 0.0;    // Current length in inches
  private int                   m_withinTolerance       = 0;      // Counter for consecutive readings within tolerance
  private double                m_extensionTotalFF;
  private boolean               m_calibrated            = EXConsts.kCalibrated;  // Indicates whether the extension has been calibrated

  private Timer                 m_safetyTimer           = new Timer( ); // Safety timer for use in extension
  private double                m_extensionDistTravelled;

  // Constructor
  public Extension2( )
  {
    setName("Extension");
    setSubsystem("Extension");

    m_extensionValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_extension, "extension", CTREConfigs6.extensionLengthFXConfig( ));

    initSmartDashboard( );

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    if (m_extensionValid)
    {
      m_extensionCurRotations = getTalonFXRotations( );

      if (m_extensionCurRotations < 0)
      {
        //m_extension.setNeutralMode(NeutralMode.Coast);
        m_extensionCurRotations = 0;
        m_extension.setRotorPosition(m_extensionCurRotations, Constants.kCANTimeoutMs);
        m_extensionTargetInches = extensionRotationsToInches(m_extensionCurRotations);
      }

      m_extensionCurInches = extensionRotationsToInches(m_extensionCurRotations);
      SmartDashboard.putNumber("EX_curInches", m_extensionCurInches);
      SmartDashboard.putNumber("EX_targetInches", m_extensionTargetInches);
      m_extensionLigament.setLength(m_extensionCurInches);

      m_extensionTotalFF = calculateTotalFF( );
      SmartDashboard.putNumber("EX_totalFF", m_extensionTotalFF);

      double currentDraw = m_extension.getStatorCurrent( ).getValue( );
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
    SmartDashboard.putNumber("EX_curInches", m_extensionCurInches);
    SmartDashboard.putNumber("EX_targetInches", m_extensionTargetInches);
    SmartDashboard.putBoolean("EX_calibrated", EXConsts.kCalibrated);

    // post the mechanism to the dashboard
    SmartDashboard.putData("ExtensionMech", m_extensionMech);
  }

  public void initialize( )
  {
    DataLogManager.log(getSubsystem( ) + ": Subsystem initialized!");

    setExtensionStopped( );

    if (m_extensionValid)
      m_extensionCurRotations = getTalonFXRotations( );
    m_extensionCurInches = extensionRotationsToInches((int) m_extensionCurRotations);
    m_extensionTargetInches = m_extensionCurInches;
    DataLogManager.log(String.format("%s: Init Target Inches: %.1f", getSubsystem( ), m_extensionTargetInches));
  }

  public double getTalonFXRotations( )
  {
    return extensionRotationsToInches(m_extension.getRotorPosition( ).refresh( ).getValue( ));
  }

  private int extensionInchesToCounts(double inches)
  {
    return (int) (inches / EXConsts.kInchesPerCount);
  }

  private int extensionInchesToRotations(double inches)
  {
    return (int) (inches / EXConsts.kRolloutRatio); //TODO: Check if valid
  }

  private int extensionRotationsToInches(double rotations)
  {
    return (int) (rotations * EXConsts.kRolloutRatio); //TODO: Check if valid
  }

  private double extensionCountsToInches(int counts)
  {
    return counts * EXConsts.kInchesPerCount;
  }

  public boolean moveIsInRange(double inches)
  {
    return (inches > EXConsts.kLengthMin) && (inches < EXConsts.kLengthMax);
  }

  public double getInches( )
  {
    return m_extensionCurInches;
  }

  public double getTalonFXInches( )
  {
    return extensionRotationsToInches(m_extension.getRotorPosition( ).refresh( ).getValue( ));
  }

  public void moveExtensionWithJoystick(XboxController joystick)
  {
    double axisValue = joystick.getRightX( );
    boolean outOfRange = false;
    ExtensionMode newMode = ExtensionMode.EXTENSION_STOPPED;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if (axisValue < 0.0)
    {
      if (m_extensionCurInches > EXConsts.kLengthMin)
        newMode = ExtensionMode.EXTENSION_IN;
      else
        outOfRange = true;
    }
    else if (axisValue > 0.0)
    {
      if (m_extensionCurInches < EXConsts.kLengthMax)
        newMode = ExtensionMode.EXTENSION_OUT;
      else
        outOfRange = true;
    }

    if (outOfRange)
      axisValue = 0.0;

    if (newMode != m_extensionMode)
    {
      m_extensionMode = newMode;
      DataLogManager.log(getSubsystem( ) + ": move " + m_extensionMode + ((outOfRange) ? " - OUT OF RANGE" : ""));
    }

    m_extensionTargetInches = m_extensionCurInches;

    if (m_extensionValid)
      m_extension.setControl(m_requestVolts.withOutput(axisValue * EXConsts.kExtensionSpeedMaxManual + EXConsts.kArbitraryFF));
  }

  public void setExtensionStopped( )
  {
    DataLogManager.log(getSubsystem( ) + ": now STOPPED");

    if (m_extensionValid)
      m_extension.setControl(m_requestVolts.withOutput(0.0));
  }

  public void moveToCalibrate( )
  {
    if (m_extensionValid)
      m_extension.setControl(m_requestVolts.withOutput(EXConsts.kSpeedCalibrate));
  }

  public void calibrate( )
  {
    if (m_extensionValid)
      m_extension.setRotorPosition(0.0);

    m_extensionTargetInches = 0;
    m_extensionCurInches = 0;
    m_calibrated = true;
    SmartDashboard.putBoolean("EX_calibrated", m_calibrated);
  }

  private double calculateTotalFF( )
  {
    double elbowDegrees = RobotContainer.getInstance( ).m_elbow.getAngle( );

    return EXConsts.kArbitraryFF * Math.cos(Math.toRadians(elbowDegrees));
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveExtensionLengthInit(ExtensionLength length)
  {
    if (length != m_extensionLength)
    {
      m_extensionLength = length;
      m_moveIsFinished = false;
      DataLogManager.log(String.format("%s: new mode request - %s", getSubsystem( ), m_extensionLength));

      switch (m_extensionLength)
      {
        default : // Fall through to NOCHANGE if invalid
          DataLogManager.log(String.format("%s: requested length is invalid - %s", getSubsystem( ), m_extensionLength));
        case EXTENSION_NOCHANGE : // Do not change from current level!
          m_extensionTargetInches = m_extensionCurInches;
          if (m_extensionTargetInches < 0.25)
            m_extensionTargetInches = 0.25;
          break;
        case EXTENSION_STOW :
          m_extensionTargetInches = EXConsts.kLengthStow;
          break;
        case EXTENSION_IDLE :
          m_extensionTargetInches = EXConsts.kLengthIdle;
          break;
        case EXTENSION_LOW :
          m_extensionTargetInches = EXConsts.kLengthScoreLow;
          break;
        case EXTENSION_MID :
          m_extensionTargetInches = EXConsts.kLengthScoreMid;
          break;
        case EXTENSION_HIGH :
          m_extensionTargetInches = EXConsts.kLengthScoreHigh;
          break;
        case EXTENSION_SHELF :
          m_extensionTargetInches = EXConsts.kLengthSubstation;
          break;
      }
    }

    if (EXConsts.kCalibrated)
    {
      // length constraint check/soft limit for max and min length before raising
      if (!moveIsInRange(m_extensionTargetInches))
      {
        DataLogManager.log(String.format("%s: Target %.1f inches is OUT OF RANGE! [%.1f, %.1f]", getSubsystem( ),
            m_extensionTargetInches, EXConsts.kLengthMin, EXConsts.kLengthMax));
        m_extensionTargetInches = m_extensionCurInches;
      }

      m_safetyTimer.restart( );

      if (m_extensionValid && EXConsts.kCalibrated)
        m_extension.setControl(m_requestMMVolts.withPosition(extensionInchesToRotations(m_extensionTargetInches)));
      //DemandType.ArbitraryFeedForward, m_extensionTotalFF); TODO: implement FF

      m_extensionDistTravelled = Math.abs(m_extensionTargetInches - m_extensionCurInches);
      DataLogManager.log(String.format("%s: moving: %.1f -> %.1f inches | counts %.1f -> %.1f", getSubsystem( ),
          m_extensionCurInches, m_extensionTargetInches, m_extensionCurInches, m_extensionTargetInches));
    }
    else
    {
      DataLogManager.log(getSubsystem( ) + ": not calibrated");
      if (m_extensionValid)
        m_extension.setControl(m_requestVolts.withOutput(0.0));
    }
  }

  public void moveExtensionLengthExecute( )
  {
    if (m_extensionValid && EXConsts.kCalibrated)
      m_extension.setControl(m_requestMMVolts.withPosition(extensionInchesToRotations(m_extensionTargetInches)));
    //DemandType.ArbitraryFeedForward, m_extensionTotalFF); TODO: implement FF
  }

  public boolean moveExtensionLengthIsFinished( )
  {
    double errorInInches = 0.0;

    errorInInches = m_extensionTargetInches - m_extensionCurInches;

    if (Math.abs(errorInInches) < EXConsts.kToleranceInches)
    {
      if (++m_withinTolerance >= 3)
      {
        m_moveIsFinished = true;
        DataLogManager.log(String.format("%s: move finished - Time: %.3f  |  Cur inches: %.1f", getSubsystem( ),
            m_safetyTimer.get( ), m_extensionCurInches));
      }
    }
    else
    {
      m_withinTolerance = 0;
    }

    if (m_safetyTimer.hasElapsed(m_extensionDistTravelled * EXConsts.kMMSafetyTimeoutRatio + 0.2))
    {
      m_moveIsFinished = true;
      DataLogManager.log(getSubsystem( ) + ": Move Safety timer has timed out! " + m_safetyTimer.get( ));
    }

    if (m_moveIsFinished)
    {
      m_withinTolerance = 0;
      m_safetyTimer.stop( );
    }

    return (m_extensionLength == ExtensionLength.EXTENSION_NOCHANGE) ? false : m_moveIsFinished;
  }

  public void moveExtensionLengthEnd( )
  {
    m_moveIsFinished = false;
    m_withinTolerance = 0;
    m_safetyTimer.stop( );
    m_extension.set(0.0);
  }

}