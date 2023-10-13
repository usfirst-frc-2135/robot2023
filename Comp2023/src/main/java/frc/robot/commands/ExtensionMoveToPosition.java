
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Extension;

/**
 *
 */
public class ExtensionMoveToPosition extends CommandBase
{
  private final Extension m_extension;
  private final Elbow     m_elbow;
  private boolean         m_holdPosition;
  private double          m_newLength;

  // Default command for holding current position
  public ExtensionMoveToPosition(Extension extension, Elbow elbow)
  {
    m_extension = extension;
    m_elbow = elbow;
    ExtensionMoveToPositionCommon(true);
  }

  // Motion Magic movement to a new position
  public ExtensionMoveToPosition(Extension extension, Elbow elbow, double position)
  {
    m_extension = extension;
    m_elbow = elbow;
    m_newLength = position;
    ExtensionMoveToPositionCommon(false);
  }

  private void ExtensionMoveToPositionCommon(boolean holdCurrentLength)
  {
    m_holdPosition = holdCurrentLength;
    setName("ExtensionMoveToPosition");
    addRequirements(m_extension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    // TODO: ZARA - double elbowAngle = m_elbow.getAngle();
    // TODO: ZARA - pass elbowAngle into moveToPositionInit (instead of m_elbow)
    m_extension.moveToPositionInit(m_newLength, m_elbow, m_holdPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    // TODO: ZARA - double elbowAngle = m_elbow.getAngle();
    // TODO: ZARA - modify moveToPositionExecute (in Extension only) to accept the elbow angle
    m_extension.moveToPositionExecute( );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_extension.moveToPositionEnd( );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return (m_holdPosition) ? false : m_extension.moveToPositionIsFinished( ); // Command exits if not holding a position
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
