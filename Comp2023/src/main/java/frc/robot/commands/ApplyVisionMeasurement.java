//
// LED Set command - sets CANdle to desired mode
//
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class ApplyVisionMeasurement extends CommandBase
{
  private final Swerve m_swerve;

  public ApplyVisionMeasurement(Swerve swerve)
  {
    m_swerve = swerve;

    setName("AppyVisionMeasurement");
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_swerve.applyVisionMeasurement(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {

  }

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
    return true;
  }
}
