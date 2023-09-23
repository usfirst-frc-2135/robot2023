//
// Apply Vision Measurement command - resets swerve odometry to a valid limelight pose
//
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VIConsts;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class DriveResetOdometry extends CommandBase
{
  private final Swerve m_swerve;
  private final Vision m_vision;

  public DriveResetOdometry(Swerve swerve, Vision vision)
  {
    m_swerve = swerve;
    m_vision = vision;

    setName("DriveResetOdometry");
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    Integer aprilTagId = RobotContainer.getInstance( ).getOdometryOption( );

    Pose2d llPose = m_vision.getLimelightRawPose( );
    if (llPose != null)
    {
      //m_swerve.resetOdometry(new Pose2d(new Translation2d(llPose.getX( ) + 2, llPose.getY( )), llPose.getRotation( ))); // TODO: ?
      m_swerve.resetLimelightOdometry(llPose);
    }
    else
    {
      if ((aprilTagId >= 1) && (aprilTagId <= 8))
      {
        m_vision.setFixedTargetID(aprilTagId);
        Pose2d atp = VIConsts.kAprilTagPoses.get(aprilTagId);
        double rotation = (aprilTagId <= 4) ? 0 : Math.PI;
        Pose2d robotPose =
            new Pose2d(new Translation2d(atp.getX( ) + ((aprilTagId <= 4) ? -3.0 : 3.0), atp.getY( )), new Rotation2d(rotation));

        DataLogManager.log(String.format("%s: Set Rotation: %.1f", getName( ), rotation));

        m_swerve.zeroGyro(Units.radiansToDegrees(rotation));
        m_swerve.resetOdometry(robotPose);
      }
    }
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
