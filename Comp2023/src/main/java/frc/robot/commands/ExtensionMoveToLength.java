
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extension;

/**
 *
 */
public class ExtensionMoveToLength extends CommandBase
{
  private final Extension m_extension;
  private double          m_length;

  public ExtensionMoveToLength(Extension extension, double length)
  {
    m_extension = extension;
    m_length = length;

    setName("ExtensionMoveToLength");
    addRequirements(m_extension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_extension.moveExtensionToPositionInit(m_length, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_extension.moveExtensionToPositionExecute( );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_extension.moveExtensionToPositionEnd( );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return m_extension.moveExtensionToPositionIsFinished( );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
