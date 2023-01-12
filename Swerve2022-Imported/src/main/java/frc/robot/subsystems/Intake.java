
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.INConsts.INMode;
import frc.robot.Constants.Ports;

/**
 *
 */
public class Intake extends SubsystemBase
{
  // Constants
  // private static final int CANTIMEOUT = 30; // CAN timeout in msec

  // Devices and simulation objects (intake was moved from CAN bus to PWM1)
  // private WPI_TalonFX m_motorIN = new WPI_TalonFX(Ports.kCANID_Intake);
  private final PWMTalonFX m_motorIN      = new PWMTalonFX(Ports.kPWM_Intake);
  private final Solenoid   m_arm          = new Solenoid(0, PneumaticsModuleType.CTREPCM, Ports.kPCM_IntakeArm);

  // @formatter:off
  // private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true, 45.0, 45.0, 0.001);
  // private StatorCurrentLimitConfiguration m_statorCurrentLimits = newStatorCurrentLimitConfiguration(true, 80.0, 80.0, 0.001);
  // @formatter:on

  // Declare module variables
  private double           m_acquireSpeed = INConsts.kINAcquireSpeed;
  private double           m_expelSpeed   = INConsts.kINExpelSpeed;

  private boolean          m_validIN      = false;

  /**
   *
   */
  public Intake( )
  {
    setName("Intake");
    setSubsystem("Intake");
    addChild("Arm", m_arm);

    SmartDashboard.putBoolean("HL_validIN", m_validIN);

    // Check if solenoid is functional or blacklisted
    DataLogManager.log(getSubsystem( ) + ": Arm Solenoid is " + ((m_arm.isDisabled( )) ? "BLACKLISTED" : "OK"));

    // Initialize Motor
    m_motorIN.setInverted(true);
    m_motorIN.setSafetyEnabled(false);
    m_motorIN.set(0.0);

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
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
    setIntakeSpeed(INMode.INTAKE_STOP);
    setArmSolenoid(false);
  }

  // Dump all Talon faults
  public void faultDump( )
  {
    // PhoenixUtil.getInstance( ).talonFXFaultDump(m_motorIN, "IN");
  }

  public void setIntakeSpeed(INMode mode)
  {
    final String strName;
    double output = 0.0; // Default: off

    switch (mode)
    {
      default :
      case INTAKE_STOP :
        strName = "STOP";
        output = 0.0;
        break;
      case INTAKE_ACQUIRE :
        strName = "ACQUIRE";
        output = m_acquireSpeed;
        break;
      case INTAKE_EXPEL :
        strName = "EXPEL";
        output = m_expelSpeed;
        break;
    }

    // Set speed of intake and the percent output
    DataLogManager.log(getSubsystem( ) + ": Set Speed - " + strName);
    m_motorIN.set(output);
  }

  public void setArmSolenoid(boolean extend)
  {
    DataLogManager.log(getSubsystem( ) + ": Arm " + ((extend) ? "DEPLOYED" : "STOWED"));
    SmartDashboard.putBoolean("IN_Deployed", extend);
    m_arm.set(extend);
  }
}
