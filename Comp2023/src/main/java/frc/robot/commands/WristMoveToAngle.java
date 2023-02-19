
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WRConsts.WristAngle;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class WristMoveToAngle extends CommandBase
{
  private final Wrist m_wrist;
  private WristAngle  m_angle;

  public WristMoveToAngle(Wrist wrist, WristAngle angle)
  {
    m_wrist = wrist;
    m_angle = angle;

    setName("WristMoveToAngle");
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_wrist.moveWristAngleInit(m_angle);
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
    return false;
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
