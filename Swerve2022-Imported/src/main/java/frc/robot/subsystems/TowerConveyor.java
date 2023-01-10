// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.TCConsts;
import frc.robot.Constants.TCConsts.TCMode;
import frc.robot.team2135.PhoenixUtil;

/**
 *
 */
public class TowerConveyor extends SubsystemBase
{
  // Constants
  private static final int                CANTIMEOUT            = 30;  // CAN timeout in msec

  // Devices and simulation objects
  private final WPI_TalonFX               m_motorTC             = new WPI_TalonFX(Ports.kCANID_TowerConv);
  private final DigitalInput              m_cargoLimit          = new DigitalInput(Ports.kDIO_CargoDetect);

  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true, 45.0, 45.0, 0.001);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 0.001);

  private boolean                         m_cargoDetected;  // current state of cargo detect limit switch

  // Declare module variables
  private double                          m_acquireSpeed        = TCConsts.kTCAcquireSpeed;
  private double                          m_acquireSpeedSlow    = TCConsts.kTCAcquireSpeedSlow;
  private double                          m_expelSpeed          = TCConsts.kTCExpelSpeed;
  private double                          m_expelSpeedFast      = TCConsts.kTCExpelSpeedFast;

  private boolean                         m_validTC             = false; // Health indicator for floor conveyor talon
  private int                             m_resetCountTC        = 0;     // reset counter for motor

  /**
   *
   */
  public TowerConveyor( )
  {
    setName("TowerConveyor");
    setSubsystem("TowerConveyor");
    addChild("CargoDetect", m_cargoLimit);

    // Validate Talon FX controllers, initialize and display firmware versions
    m_validTC = PhoenixUtil.getInstance( ).talonFXInitialize(m_motorTC, "TC");
    SmartDashboard.putBoolean("HL_validTC", m_validTC);

    // Initialize Motor
    if (m_validTC)
    {
      m_motorTC.setInverted(false);
      m_motorTC.setNeutralMode(NeutralMode.Coast);
      m_motorTC.set(ControlMode.PercentOutput, 0.0);

      m_motorTC.configSupplyCurrentLimit(m_supplyCurrentLimits);
      m_motorTC.configStatorCurrentLimit(m_statorCurrentLimits);
      m_motorTC.setStatusFramePeriod(StatusFrame.Status_1_General, 255, CANTIMEOUT);
      m_motorTC.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255, CANTIMEOUT);
    }

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    if (m_motorTC.hasResetOccurred( ))
    {
      m_resetCountTC += 1;
      SmartDashboard.putNumber("HL_resetCountTC", m_resetCountTC);
    }

    m_cargoDetected = isCargoDetected( );
    SmartDashboard.putBoolean("TC_cargoDetected", m_cargoDetected);
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run when in simulation
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {
    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");
    setTowerConveyorSpeed(TCMode.TCONVEYOR_STOP);
  }

  public void faultDump( )
  {
    PhoenixUtil.getInstance( ).talonFXFaultDump(m_motorTC, "TC");
  }

  // Set mode of conveyor
  public void setTowerConveyorSpeed(TCMode mode)
  {
    final String strName;
    double output = 0.0; // Default: off

    switch (mode)
    {
      default :
      case TCONVEYOR_STOP :
        strName = "STOP";
        output = 0.0;
        break;
      case TCONVEYOR_ACQUIRE :
        strName = "ACQUIRE";
        output = m_acquireSpeed;
        break;
      case TCONVEYOR_ACQUIRE_SLOW :
        strName = "ACQUIRE_SLOW";
        output = m_acquireSpeedSlow;
        break;
      case TCONVEYOR_EXPEL :
        strName = "EXPEL";
        output = m_expelSpeed;
        break;
      case TCONVEYOR_EXPEL_FAST :
        strName = "EXPEL_FAST";
        output = m_expelSpeedFast;
        break;
    }

    DataLogManager.log(getSubsystem( ) + ": TC Set Speed - " + strName);
    if (m_validTC)
      m_motorTC.set(ControlMode.PercentOutput, output);
  }

  public boolean isCargoDetected( )
  {
    m_cargoDetected = m_cargoLimit.get( );
    return m_cargoDetected;
  }
}
