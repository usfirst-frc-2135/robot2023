
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class DriveQuickturn extends CommandBase
{
  private final Drivetrain m_drivetrain;

  public DriveQuickturn(Drivetrain drivetrain)
  {
    m_drivetrain = drivetrain;

    setName("DriveQuickturn");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_drivetrain.driveSetQuickTurn(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_drivetrain.driveSetQuickTurn(false);
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
