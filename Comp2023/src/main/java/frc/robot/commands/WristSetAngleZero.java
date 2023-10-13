
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class WristSetAngleZero extends CommandBase
{
  private final Wrist m_wrist;

  public WristSetAngleZero(Wrist wrist)
  {
    m_wrist = wrist;

    setName("WristSetAngleZero");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_wrist.resetPositionToZero( );
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
