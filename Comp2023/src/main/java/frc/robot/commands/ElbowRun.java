package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elbow;

/**
 *
 */
public class ElbowRun extends CommandBase
{
  private final Elbow    m_elbow;

  private XboxController m_gamePad;

  public ElbowRun(Elbow elbow, XboxController gamePad)
  {
    m_elbow = elbow;
    m_gamePad = gamePad;

    setName("ElbowRun");
    addRequirements(m_elbow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_elbow.moveElbowWithJoystick(m_gamePad);
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
