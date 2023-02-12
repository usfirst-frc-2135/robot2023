
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ARMConsts.WristHeight;
import frc.robot.subsystems.Arm;

/**
 *
 */
public class WristMoveToHeight extends CommandBase
{
  private final Arm   m_wrist;
  private WristHeight m_height;

  public WristMoveToHeight(Arm wrist, WristHeight height)
  {
    m_wrist = wrist;
    m_height = height;

    setName("ClimberMoveToHeight");
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_wrist.moveWristDistanceInit(m_height);
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
