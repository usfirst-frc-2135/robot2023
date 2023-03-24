package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WRConsts;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class WristRunConstant extends CommandBase
{

  private final Wrist   m_wrist;
  private final boolean m_moveUp;

  public WristRunConstant(Wrist wrist, boolean moveUp)
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
    m_wrist.moveWristConstantSpeed(WRConsts.kWristSpeedMaxManual * (m_moveUp ? 1 : -1));
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
