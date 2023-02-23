//
// Apply Vision Measurement command - resets swerve odometry to a valid limelight pose
//
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class ResetOdometryToLimelight extends CommandBase
{
  private final Swerve m_swerve;
  private final Vision m_vision;
  private final int    m_id;

  public ResetOdometryToLimelight(Swerve swerve, Vision vision, int id)
  {
    m_swerve = swerve;
    m_vision = vision;
    m_id = id;

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
    else if ((m_id > 0) && (m_id < 9))
    {
      m_vision.setFixedTargetID(m_id);
      Pose2d atp = Constants.VIConsts.kAprilTagPoses.get(m_id);
      double rotation = (m_id <= 4) ? Math.PI : -Math.PI;
      Pose2d robotPose =
          new Pose2d(new Translation2d(atp.getX( ) + ((m_id <= 4) ? -3.0 : 3.0), atp.getY( )), new Rotation2d(rotation));

      DataLogManager.log(String.format("Set Rotation %.2f", rotation));

      m_swerve.resetOdometry(robotPose);
    }
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
