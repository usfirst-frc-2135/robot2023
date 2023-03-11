
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elbow;

/**
 *
 */
public class SetELAngleZero extends CommandBase
{
  private final Elbow m_elbow;

  public SetELAngleZero(Elbow elbow)
  {
    m_elbow = elbow;

    setName("SetELAngleZero");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_elbow.setElbowAngleToZero( );
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
