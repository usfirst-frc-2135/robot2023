
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 *
 */
public class ShooterReverse extends CommandBase
{
  private final Shooter m_shooter;

  public ShooterReverse(Shooter shooter)
  {
    m_shooter = shooter;

    setName("ShooterReverse");
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_shooter.setReverseInit( );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_shooter.setReverseExecute( );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_shooter.setReverseEnd( );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return false;
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
