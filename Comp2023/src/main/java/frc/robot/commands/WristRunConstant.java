package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist2;

/**
 *
 */
public class WristRunConstant extends CommandBase
{

  private final Wrist2  m_wrist;
  private final boolean m_moveUp;

  public WristRunConstant(Wrist2 wrist, boolean moveUp)
  {
    m_wrist = wrist;
    m_moveUp = moveUp;

    setName("WristRunConstant");
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_wrist.moveWristConstantSpeed(m_moveUp ? 2.0 : -2.0);
  }

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
