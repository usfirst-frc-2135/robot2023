
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extension;

/**
 *
 */
public class ExtensionCalibrate extends CommandBase
{
  private Timer     m_calibrateTimer = new Timer( );
  private Extension m_extension;

  public ExtensionCalibrate(Extension extension)
  {
    m_extension = extension;

    setName("ExtensionCalibrate");
    addRequirements(m_extension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_calibrateTimer.reset( );
    m_extension.moveToCalibrate( );
    m_calibrateTimer.start( );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_calibrateTimer.stop( );
    m_extension.calibrate( );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return m_calibrateTimer.hasElapsed(1.0);
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
