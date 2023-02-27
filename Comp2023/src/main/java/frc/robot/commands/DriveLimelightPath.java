//
// Drive Limelight path - uses a limelight target to generate a trajectory and follow it
//
package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VIConsts;
import frc.robot.Constants.VIConsts.VIGoalDirection;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class DriveLimelightPath extends CommandBase
{
  private final Swerve          m_swerve;
  private final Vision          m_vision;
  private final VIGoalDirection m_goalDirection;
  private Pose2d                m_goalPose;

  public DriveLimelightPath(Swerve swerve, Vision vision, VIGoalDirection goalDirection)
  {
    m_swerve = swerve;
    m_vision = vision;
    m_goalDirection = goalDirection;

    setName("DriveLimelight");
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    Pose2d currentPose = m_swerve.getPose( );
    DataLogManager.log(getName( ) + ": goalDirection " + m_goalDirection + " curPose " + currentPose);

    m_goalPose = getGoalPose(m_goalDirection);
    DataLogManager.log(getName( ) + ": goalPose " + m_goalPose);

    if (m_goalPose != null)
    {
      PathPlannerTrajectory trajectory = PathPlanner.generatePath(new PathConstraints(3, 4),
          new PathPoint(currentPose.getTranslation( ), currentPose.getRotation( ), currentPose.getRotation( )),
          new PathPoint(m_goalPose.getTranslation( ), m_goalPose.getRotation( ), m_goalPose.getRotation( )));

      m_swerve.driveWithPathFollowerInit(trajectory, true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    if (m_goalPose != null)
      m_swerve.driveWithPathFollowerExecute( );
  }

  // Called once the command ends or is  interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_swerve.driveWithPathFollowerEnd( );
    if (m_goalPose != null)
      m_swerve.driveStop(true);
  }

  // Returns true when the command  sh
  @Override
  public boolean isFinished( )
  {
    return (m_goalPose == null) || m_swerve.driveWithPathFollowerIsFinished( );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }

  public Pose2d getGoalPose(VIConsts.VIGoalDirection goalDirection)
  {
    int targetId = m_vision.getTargetID( );

    if (!m_vision.isAprilTagValid(targetId))
      return null;

    Pose2d targetPose = VIConsts.kAprilTagPoses.get(targetId);
    double goalXValue = 0;
    double goalYValue = 0;
    String strName;

    // TODO: Need to add a named constant for the X distance from AprilTag location to robot center
    goalXValue = targetPose.getX( ) + ((targetId <= 4) ? -1.0 : 1.0);

    // TODO: Need to add a named constant for the left/right offset to align with cone poles
    switch (goalDirection)
    {
      default :
      case DIRECTION_LEFT :
        strName = "LEFT";
        goalYValue = targetPose.getY( ) + ((targetId <= 4) ? -1.0 : 1.0);
        break;
      case DIRECTION_MIDDLE :
        strName = "MIDDLE";
        goalYValue = targetPose.getY( );
        break;
      case DIRECTION_RIGHT :
        strName = "RIGHT";
        goalYValue = targetPose.getY( ) + ((targetId <= 4) ? 1.0 : -1.0);
        break;
    }

    DataLogManager.log(String.format("Calculate target ID %d direction %s", targetId, strName));

    return new Pose2d(new Translation2d(goalXValue, goalYValue),
        new Rotation2d(targetPose.getRotation( ).getRadians( ) + Math.PI));
  }

}
