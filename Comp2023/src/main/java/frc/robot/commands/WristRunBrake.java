package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WRConsts;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class WristRunBrake extends CommandBase
{

  private final Wrist   m_wrist;
  private final Boolean m_brakeUp;
  private Timer         m_timer = new Timer( );

  public WristRunBrake(Wrist wrist, boolean brakeUp)
  {
    m_wrist = wrist;
    m_brakeUp = brakeUp;

    setName("WristRunBrake");
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_timer.restart( );
    m_wrist.moveConstantSpeed(m_brakeUp ? -WRConsts.kBrakeSpeedVolts : WRConsts.kBrakeSpeedVolts);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return m_timer.hasElapsed(0.250);
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
