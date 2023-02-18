
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ARMConsts.ElbowAngle;
import frc.robot.subsystems.Arm;

/**
 *
 */
public class ElbowMoveToAngle extends CommandBase
{
  private final Arm  m_elbow;
  private ElbowAngle m_Angle;

  public ElbowMoveToAngle(Arm elbow, ElbowAngle angle)
  {
    m_elbow = elbow;
    m_angle = angle;

    setName("ElbowMoveToAngle");
    addRequirements(m_elbow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_elbow.moveElbowDistanceInit(m_angle);
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
