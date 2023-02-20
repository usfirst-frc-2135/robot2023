//
// LED Set command - sets CANdle to desired mode
//
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VIConsts;
import frc.robot.Constants.VIConsts.VITargetLocations;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class DriveLimelight extends CommandBase
{
  private final Swerve m_swerve;
  private final VITargetLocations    m_targetLocation;

  public DriveLimelight(Swerve swerve, VITargetLocations targetLocation)
  {
    m_swerve = swerve;
    m_targetLocation = targetLocation;

    setName("DriveLimelight");
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_swerve.driveWithLimelightInit(m_swerve.calculateTarget(m_targetLocation));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_swerve.driveWithLimelightExecute( );
  }

  // Called once the command ends or is  interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_swerve.driveWithLimelightEnd( );
  }

  // Returns true when the command  sh
  @Override
  public boolean isFinished( )
  {
    return m_swerve.driveWithLimelightIsFinished( );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return true;
  }
}
