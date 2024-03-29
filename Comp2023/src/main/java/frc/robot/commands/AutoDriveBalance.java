package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class AutoDriveBalance extends CommandBase
{
  private Swerve m_swerve;
  private Timer  m_timer = new Timer( );

  public AutoDriveBalance(Swerve swerve)
  {
    m_swerve = swerve;

    setName("AutoDriveBalance");
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_timer.restart( );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_swerve.driveBalanceExecute( );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_timer.stop( );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return (m_timer.hasElapsed(10.0) || RobotState.isDisabled( ));
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
