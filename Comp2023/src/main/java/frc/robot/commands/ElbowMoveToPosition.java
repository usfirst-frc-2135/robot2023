
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ELConsts.ElbowPosition;
import frc.robot.subsystems.Elbow;

/**
 *
 */
public class ElbowMoveToPosition extends CommandBase
{
  private final Elbow   m_elbow;
  private ElbowPosition m_position;

  public ElbowMoveToPosition(Elbow elbow, ElbowPosition position)
  {
    m_elbow = elbow;
    m_position = position;

    setName("ElbowMoveToPosition");
    addRequirements(m_elbow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_elbow.moveElbowToPositionInit(m_position);
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
    return m_elbow.moveElbowToPositionIsFinished( );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
