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
import edu.wpi.first.math.util.Units;
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

  public DriveLimelightPath(Swerve swerve, Vision vision, VIGoalDirection goalDirection)
  {
    m_swerve = swerve;
    m_vision = vision;
    m_goalDirection = goalDirection;

    setName("DriveLimelightPath");
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    Pose2d goalPose2d = calculateTarget(m_vision.getTargetID( ), m_goalDirection);
    Pose2d currentPose = m_swerve.getPose( );

    PathPlannerTrajectory trajectory = PathPlanner.generatePath(new PathConstraints(3, 4),
        new PathPoint(currentPose.getTranslation( ), currentPose.getRotation( ), currentPose.getRotation( )),
        new PathPoint(goalPose2d.getTranslation( ), goalPose2d.getRotation( ), goalPose2d.getRotation( )));

    m_swerve.driveWithPathFollowerInit(trajectory, true);

    DataLogManager.log(String.format("%s: current %s, goal %s", getName( ), currentPose, goalPose2d));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_swerve.driveWithPathFollowerExecute( );
  }

  // Called once the command ends or is  interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_swerve.driveWithPathFollowerEnd( );
    m_swerve.driveStop(true);
  }

  // Returns true when the command  sh
  @Override
  public boolean isFinished( )
  {
    return m_swerve.driveWithPathFollowerIsFinished( );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }

  public Pose2d calculateTarget(int targetId, VIConsts.VIGoalDirection goalDirection)
  {
    Pose2d targetPose = VIConsts.kAprilTagPoses.get(targetId);
    double goalXValue = 0;
    double goalYValue = 0;
    String strName;

    goalXValue = targetPose.getX( ) + ((targetId <= 4) ? -1.0 : 1.0);

    switch (goalDirection)
    {
      default :
      case DIRECTION_LEFT :
        strName = "LEFT";
        goalYValue = targetPose.getY( ) + ((targetId <= 4) ? -1 : 1);
        break;
      case DIRECTION_MIDDLE :
        strName = "MIDDLE";
        goalYValue = targetPose.getY( );
        break;
      case DIRECTION_RIGHT :
        strName = "RIGHT";
        goalYValue = targetPose.getY( ) + ((targetId <= 4) ? 1 : -1);
        break;
    }

    DataLogManager.log(String.format("%s: Calculate target ID %d direction %s", getName( ), targetId, strName));

    return new Pose2d(new Translation2d(goalXValue, goalYValue), new Rotation2d(targetPose.getRotation( ).getRadians( ) + 3.14));

  }
}
