
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TCConsts.TCMode;
import frc.robot.subsystems.TowerConveyor;

/**
 *
 */
public class TowerConveyorRun extends CommandBase
{
  private final TowerConveyor m_towerConveyor;
  private final TCMode        m_mode;

  public TowerConveyorRun(TowerConveyor towerConveyor, TCMode mode)
  {
    m_towerConveyor = towerConveyor;
    m_mode = mode;

    setName("TowerConveyorRun");
    addRequirements(m_towerConveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_towerConveyor.setTowerConveyorSpeed(m_mode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return true;
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
