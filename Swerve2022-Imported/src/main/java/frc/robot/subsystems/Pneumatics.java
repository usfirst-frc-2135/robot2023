
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *
 */
public class Pneumatics extends SubsystemBase
{
  private final PneumaticsControlModule m_pcm        = new PneumaticsControlModule(0);
  private final Compressor              m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  /**
   *
   */
  public Pneumatics( )
  {
    setName("Pneumatics");
    setSubsystem("Pneumatics");
    addChild("Compressor", m_compressor);

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("PCM_Output_Comp", m_pcm.getCompressorCurrent( ));
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
  }

  public void faultDump( )
  {
    // Print out PCM faults and clear sticky ones
    DataLogManager.log(getSubsystem( ) + ": ----- PCM FAULTS --------------");

    if (m_pcm.getCompressorCurrentTooHighFault( ))
      DataLogManager.log(getSubsystem( ) + ": Warn - CurrentTooHighFault");
    if (m_pcm.getCompressorNotConnectedFault( ))
      DataLogManager.log(getSubsystem( ) + ": Warn - CompressorNotConnectedFault");
    if (m_pcm.getCompressorShortedFault( ))
      DataLogManager.log(getSubsystem( ) + ": Warn - CompressorShortedFault");
    if (m_pcm.getSolenoidVoltageFault( ))
      DataLogManager.log(getSubsystem( ) + ": Warn - SolenoidVoltageFault");

    m_pcm.clearAllStickyFaults( );
  }
}
