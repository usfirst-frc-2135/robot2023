package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class WristRun extends CommandBase
{

  private final Wrist    m_wrist;

  private XboxController m_gamePad;

  public WristRun(Wrist wrist, XboxController gamePad)
  {
    m_wrist = wrist;
    m_gamePad = gamePad;

    setName("WristRun");
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
    m_wrist.moveWristWithJoystick(m_gamePad);
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
