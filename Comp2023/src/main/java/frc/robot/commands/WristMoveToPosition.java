// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class WristMoveToPosition extends CommandBase
{
  private final Wrist m_wrist;
  private boolean     m_holdPosition;
  private double      m_newAngle;

  // Default command for holding current position
  public WristMoveToPosition(Wrist wrist)
  {
    m_wrist = wrist;
    WristMoveToPositionCommon(true);
  }

  // Motion Magic movement to a new position
  public WristMoveToPosition(Wrist wrist, double position)
  {
    m_wrist = wrist;
    m_newAngle = position;
    WristMoveToPositionCommon(false);
  }

  private void WristMoveToPositionCommon(boolean holdCurrentAngle)
  {
    m_holdPosition = holdCurrentAngle;
    setName("WristMoveToPosition");
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_wrist.moveWristToPositionInit(m_newAngle, m_holdPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_wrist.moveWristToPositionExecute( );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_wrist.moveWristToPositionEnd( );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return (m_holdPosition) ? false : m_wrist.moveWristToPositionIsFinished( ); // Command exits if not holding a position
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
