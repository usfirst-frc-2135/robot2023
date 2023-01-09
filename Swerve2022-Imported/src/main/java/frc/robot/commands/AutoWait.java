
// ROBOTBUILDER TYPE: WaitCommand.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AUTOConstants.AutoTimer;

/**
 *
 */
public class AutoWait extends CommandBase
{
  private AutoTimer m_timerNum;
  private Timer     m_timer = new Timer( );
  private double    m_waitTime;

  public AutoWait(AutoTimer timerNum)
  {
    m_timerNum = timerNum;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize( )
  {
    if (m_timerNum == AutoTimer.TIMER1)
      m_waitTime = SmartDashboard.getNumber("AUTO_WaitTime1", 0.0);

    if (m_timerNum == AutoTimer.TIMER2)
      m_waitTime = SmartDashboard.getNumber("AUTO_WaitTime2", 0.0);

    m_timer.reset( );
    m_timer.start( );
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute( )
  {}

  @Override
  public boolean isFinished( )
  {
    return m_timer.hasElapsed(m_waitTime);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted)
  {
    m_timer.stop( );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
