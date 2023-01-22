
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

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
  private final Swerve  m_swerve;
  private final boolean m_useInitialPose;

  private String        m_trajectoryJSON;
  private Trajectory    m_trajectory;

  public AutoDrivePath(Swerve drivetrain, String pathName, boolean useInitialPose)
  {
    m_swerve = drivetrain;
    m_useInitialPose = useInitialPose;

    setName("AutoDrivePath");
    addRequirements(m_swerve);

    // Get our trajectory
    m_trajectoryJSON = Filesystem.getDeployDirectory( ).getAbsolutePath( ).toString( ) + "/output/" + pathName + ".wpilib.json";

    try
    {
      DataLogManager.log(String.format("%s: TrajPath %s", getName( ), m_trajectoryJSON));
      Path trajectoryPath = Filesystem.getDeployDirectory( ).toPath( ).resolve(m_trajectoryJSON);
      m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      DataLogManager.log(String.format("%s: Num states %2d Total time %.3f secs", getName( ), m_trajectory.getStates( ).size( ),
          m_trajectory.getTotalTimeSeconds( )));
    }
    catch (IOException ex)
    {
      DataLogManager.log(String.format("%s: Unable to open %s", getName( ), m_trajectoryJSON));
      DriverStation.reportError("Unable to open trajectory: " + m_trajectoryJSON, ex.getStackTrace( ));
    }

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
