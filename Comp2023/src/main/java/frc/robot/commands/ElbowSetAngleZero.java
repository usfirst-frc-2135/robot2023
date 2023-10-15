
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elbow;

/**
 *
 */
public class ElbowSetAngleZero extends CommandBase
{
  private final Elbow m_elbow;

  public ElbowSetAngleZero(Elbow elbow)
  {
    m_elbow = elbow;

    setName("ElbowSetAngleZero");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_elbow.resetPositionToZero( );
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
