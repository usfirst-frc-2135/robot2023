package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class DriveBalance extends CommandBase
{
  private Swerve m_swerve;
  private Timer  m_timer = new Timer( );

  public DriveBalance(Swerve swerve)
  {
    m_swerve = swerve;

    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    DataLogManager.log("Running Drive Balance!");
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
    return (m_timer.hasElapsed(15) || RobotState.isDisabled( ));
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
