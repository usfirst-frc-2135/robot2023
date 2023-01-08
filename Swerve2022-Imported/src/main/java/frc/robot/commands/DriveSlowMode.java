
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class DriveSlowMode extends CommandBase
{
  private final Drivetrain m_drivetrain;
  private final boolean    m_driveSlow;

  public DriveSlowMode(Drivetrain drivetrain, boolean driveSlow)
  {
    m_drivetrain = drivetrain;
    m_driveSlow = driveSlow;

    setName("DriveSlowMode");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_drivetrain.setDriveSlowMode(m_driveSlow);
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
    return true;
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
