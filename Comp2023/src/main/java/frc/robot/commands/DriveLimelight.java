//
// LED Set command - sets CANdle to desired mode
//
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class DriveLimelight extends CommandBase
{
  private final Swerve m_swerve;
  private final int    m_targetLocation;
  private final Pose2d m_goalPose2d;

  public DriveLimelight(Swerve swerve, int targetLocation)
  {
    m_swerve = swerve;
    m_targetLocation = targetLocation;
    m_goalPose2d = m_swerve.calculateTarget(m_targetLocation);

    setName("DriveLimelight");
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_swerve.driveWithLimelightInit(m_goalPose2d);
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
    return true;
  }
}
