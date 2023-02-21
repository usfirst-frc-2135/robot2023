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
import frc.robot.Constants;
import frc.robot.Constants.VIConsts;
import frc.robot.Constants.VIConsts.VITargetLocations;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class DriveLimelightPath extends CommandBase
{
  private final Swerve            m_swerve;
  private final Vision            m_vision;
  private final VITargetLocations m_targetLocation;

  public DriveLimelightPath(Swerve swerve, Vision vision, VITargetLocations targetLocation)
  {
    m_swerve = swerve;
    m_vision = vision;
    m_targetLocation = targetLocation;

    setName("DriveLimelight");
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    Pose2d goalPose2d = calculateTarget(m_targetLocation);
    Pose2d currentPose = m_swerve.getPose( );

    PathPlannerTrajectory trajectory = PathPlanner.generatePath(new PathConstraints(3, 4),
        new PathPoint(currentPose.getTranslation( ), goalPose2d.getRotation( )),
        new PathPoint(goalPose2d.getTranslation( ), goalPose2d.getRotation( )));

    m_swerve.driveWithPathFollowerInit(trajectory, true);

    DataLogManager.log("GOAL POSE@D +++ " + goalPose2d);
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

  public Pose2d calculateTarget(VIConsts.VITargetLocations targetLocation)
  {
    int targetID = m_vision.getTargetID( );
    Pose2d aprilTagPose2d = Constants.VIConsts.kAprilTagPoses.get(targetID);
    double targetXvalue = 0;
    String strName;

    switch (targetLocation)
    {
      default :
      case TARGET_MIDDLE :
        strName = "MIDDLE";
        targetXvalue = aprilTagPose2d.getX( ) + 2;
        break;
    }
    return new Pose2d(new Translation2d(targetXvalue, aprilTagPose2d.getY( )), new Rotation2d(0));
  }
}
