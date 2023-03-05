
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class AutoDrivePath extends CommandBase
{
  private final Swerve          m_swerve;
  private final boolean         m_useInitialPose;

  private PathPlannerTrajectory m_trajectory;
  private String                m_pathName;

  public AutoDrivePath(Swerve swerve, String pathName, boolean useInitialPose)
  {
    m_swerve = swerve;
    m_useInitialPose = useInitialPose;
    m_pathName = pathName;

    setName("AutoDrivePath");
    addRequirements(m_swerve);

    // Get our trajectory
    m_trajectory = PathPlanner.loadPath(pathName, new PathConstraints(4, 3));
    DataLogManager.log(String.format("%s: '%s' has %2d states Total time - %.3f secs", getName( ), pathName,
        m_trajectory.getStates( ).size( ), m_trajectory.getTotalTimeSeconds( )));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Running '%s'", getName( ), m_pathName));
    m_swerve.driveWithPathFollowerInit(m_trajectory, m_useInitialPose);
    m_swerve.setAutoMode(true);
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
