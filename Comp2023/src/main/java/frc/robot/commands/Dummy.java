
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *
 */
public class Dummy extends CommandBase
{
  private final String m_string;

  public Dummy(String string)
  {
    m_string = string;

    setName("Dummy");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: key pressed - %s", getName( ), m_string));
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
    return true;
  }
}
