
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ARMConsts.ElbowHeight;
import frc.robot.subsystems.Arm;

/**
 *
 */
public class ElbowMoveToHeight extends CommandBase
{
  private final Arm   m_elbow;
  private ElbowHeight m_height;

  public ElbowMoveToHeight(Arm elbow, ElbowHeight height)
  {
    m_elbow = elbow;
    m_height = height;

    setName("ElbowMoveToHeight");
    addRequirements(m_elbow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_elbow.moveElbowDistanceInit(m_height);
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
