
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class DriveMotorTest extends CommandBase
{
  private final Swerve        m_swerve;
  private final boolean       m_left;

  private static final double m_voltsToRun = 3.0;

  public DriveMotorTest(Swerve swerve, boolean left)
  {
    m_swerve = swerve;
    m_left = left;

    setName("DriveMotorTest");
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    //TODO: ADD BACK IN
    //m_swerve.tankDriveVolts(m_left ? m_voltsToRun : 0.0, m_left ? 0.0 : m_voltsToRun);
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
