
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

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
  private String                m_pathName;
  private PathPlannerTrajectory m_trajectory;
  private final boolean         m_useInitialPose;

  public AutoDrivePath(Swerve swerve, String pathName, PathPlannerTrajectory trajectory, boolean useInitialPose)
  {
    m_swerve = swerve;
    m_pathName = pathName;
    m_trajectory = trajectory;
    m_useInitialPose = useInitialPose;

    setName("AutoDrivePath");
    addRequirements(m_swerve);

    // Get our trajectory
    DataLogManager.log(String.format("%s: '%s' has %2d states Total time - %.3f secs", getName( ), pathName,
        m_trajectory.getStates( ).size( ), m_trajectory.getTotalTimeSeconds( )));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Running '%s'", getName( ), m_pathName));
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
