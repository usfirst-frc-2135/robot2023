//
// Apply Vision Measurement command - resets swerve odometry to a valid limelight pose
//
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class ApplyVisionMeasurement extends CommandBase
{
  private final Swerve m_swerve;
  private final Vision m_vision;

  public ApplyVisionMeasurement(Swerve swerve, Vision vision)
  {
    m_swerve = swerve;
    m_vision = vision;

    setName("AppyVisionMeasurement");
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    Pose2d llPose = m_vision.getLimelightRawPose( );
    if (llPose != null)
      m_swerve.resetOdometry(llPose);
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
