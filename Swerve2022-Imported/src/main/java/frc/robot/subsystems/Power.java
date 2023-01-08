
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *
 */
public class Power extends SubsystemBase
{
  private final PowerDistribution m_powerDistribution = new PowerDistribution( );;

  /**
   *
   */
  public Power( )
  {
    setName("Power");
    addChild("PowerDistribution", m_powerDistribution);

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
    DataLogManager.log(getSubsystem( ) + ": Init Voltage is " + String.format("%.1f", m_powerDistribution.getVoltage( )));
  }

  public void faultDump( )
  {
    DataLogManager.log(getSubsystem( ) + ": Temperature is " + m_powerDistribution.getTemperature( ));
    DataLogManager.log(getSubsystem( ) + ": Input Voltage is " + m_powerDistribution.getVoltage( ));
    for (int i = 0; i <= 15; i++)
    {
      DataLogManager.log(getSubsystem( ) + ": Chan is " + i + " Current is " + m_powerDistribution.getCurrent(i));
    }
    DataLogManager.log(getSubsystem( ) + ": Total Current is " + m_powerDistribution.getTotalCurrent( ));
    DataLogManager.log(getSubsystem( ) + ": Total Power is " + m_powerDistribution.getTotalPower( ));
    DataLogManager.log(getSubsystem( ) + ": Total Energy is " + m_powerDistribution.getTotalEnergy( ));

    m_powerDistribution.resetTotalEnergy( );
    m_powerDistribution.clearStickyFaults( );
  }
}
