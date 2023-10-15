//
// Power subystem - handles Power Distribution Hub readings
//
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//
// Power subsystem class
//
public class Power extends SubsystemBase
{
  // Member objects
  private final PowerDistribution m_powerDistribution = new PowerDistribution( );;

  public Power( )
  {
    setName("Power");
    setSubsystem("Power");

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
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));
    DataLogManager.log(String.format("%s: Init Voltage is %.1f", getSubsystem( ), m_powerDistribution.getVoltage( )));
  }

  public void faultDump( )
  {
    DataLogManager.log(String.format("%s: Temperature is %.1f", getSubsystem( ), m_powerDistribution.getTemperature( )));
    DataLogManager.log(String.format("%s: Input Voltage is %.1f volts", getSubsystem( ), m_powerDistribution.getVoltage( )));
    for (int i = 0; i <= 15; i++)
    {
      DataLogManager
          .log(String.format("%s: Chan is %d Current is %.1f amps", getSubsystem( ), i, m_powerDistribution.getCurrent(i)));
    }
    DataLogManager.log(String.format("%s: Total Current is %.1f", getSubsystem( ), m_powerDistribution.getTotalCurrent( )));
    DataLogManager.log(String.format("%s: Total Power is %.1f watts", getSubsystem( ), m_powerDistribution.getTotalPower( )));
    DataLogManager.log(String.format("%s: Total Energy is %.1f joules", getSubsystem( ), m_powerDistribution.getTotalEnergy( )));

    m_powerDistribution.resetTotalEnergy( );
    m_powerDistribution.clearStickyFaults( );
  }
}
