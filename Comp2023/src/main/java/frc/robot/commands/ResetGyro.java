
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class ResetGyro extends CommandBase
{
  private final Swerve         swerve;
  private final JoystickButton m_start;
  private final JoystickButton m_back;

  public ResetGyro(Swerve drive, JoystickButton start, JoystickButton back)
  {
    swerve = drive;
    m_start = start;
    m_back = back;
    setName("ResetGyro");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    DataLogManager.log(String.format("%s Init", getName( )));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    if (m_start.getAsBoolean( ) && m_back.getAsBoolean( ))
      swerve.zeroGyro( );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return true;
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
