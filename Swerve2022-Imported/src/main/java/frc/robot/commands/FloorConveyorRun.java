
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FCConsts.FCMode;
import frc.robot.subsystems.FloorConveyor;

/**
 *
 */
public class FloorConveyorRun extends CommandBase
{
  private final FloorConveyor m_floorConveyor;
  private final FCMode        m_mode;

  public FloorConveyorRun(FloorConveyor floorConveyor, FCMode mode)
  {
    m_floorConveyor = floorConveyor;
    m_mode = mode;

    setName("FloorConveyorRun");
    addRequirements(m_floorConveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_floorConveyor.setFloorConveyorSpeed(m_mode);
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
