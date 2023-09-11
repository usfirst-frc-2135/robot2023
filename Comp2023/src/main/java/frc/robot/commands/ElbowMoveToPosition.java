
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elbow;

/**
 *
 */
public class ElbowMoveToPosition extends CommandBase
{
  private final Elbow m_elbow;
  private boolean     m_holdPosition;
  private double      m_newAngle;

  // Default command for holding current position
  public ElbowMoveToPosition(Elbow elbow)
  {
    m_elbow = elbow;
    ElbowMoveToPositionCommon(true);
  }

  // Motion Magic movement to a new position
  public ElbowMoveToPosition(Elbow elbow, double position)
  {
    m_elbow = elbow;
    m_newAngle = position;
    ElbowMoveToPositionCommon(false);
  }

  private void ElbowMoveToPositionCommon(boolean holdCurrentAngle)
  {
    m_holdPosition = holdCurrentAngle;
    setName("ElbowMoveToPosition");
    addRequirements(m_elbow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_elbow.moveElbowToPositionInit(m_newAngle, m_holdPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_elbow.moveElbowToPositionExecute( );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_elbow.moveElbowToPositionEnd( );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return (m_holdPosition) ? false : m_elbow.moveElbowToPositionIsFinished( ); // Command exits if not holding a position
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
