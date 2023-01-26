
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class AutoDrivePath extends CommandBase
{
  private final Swerve          m_swerve;
  private final boolean         m_useInitialPose;

  private String                m_trajectoryJSON;
  private PathPlannerTrajectory m_trajectory;

  public AutoDrivePath(Swerve drivetrain, String pathName, boolean useInitialPose)
  {
    m_swerve = drivetrain;
    m_useInitialPose = useInitialPose;

    setName("AutoDrivePath");
    addRequirements(m_swerve);

    // Get our trajectory
    m_trajectoryJSON = Filesystem.getDeployDirectory( ).getAbsolutePath( ).toString( ) + "/pathplanner/" + pathName + ".path";

    //DataLogManager.log(String.format("%s: TrajPath %s", getName( ), m_trajectoryJSON));
    m_trajectory = PathPlanner.loadPath(pathName, new PathConstraints(4, 3));
    DataLogManager.log(String.format("%s: Num states %2d Total time %.3f secs", getName( ), m_trajectory.getStates( ).size( ),
        m_trajectory.getTotalTimeSeconds( )));

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Running", getName( ), m_trajectoryJSON));
    m_swerve.driveWithPathFollowerInit(m_trajectory, m_useInitialPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_swerve.driveWithPathFollowerExecute( );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_swerve.driveWithPathFollowerEnd( );
  }

  // Returns true when the command should end.
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
}
