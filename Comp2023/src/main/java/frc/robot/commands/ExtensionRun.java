package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extension;

/**
 *
 */
public class ExtensionRun extends CommandBase
{
  private final Extension m_extension;

  private XboxController  m_gamePad;

  public ExtensionRun(Extension extension, XboxController gamePad)
  {
    m_extension = extension;
    m_gamePad = gamePad;

    setName("ExtensionRun");
    addRequirements(m_extension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_extension.moveExtensionWithJoystick(m_gamePad);
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
