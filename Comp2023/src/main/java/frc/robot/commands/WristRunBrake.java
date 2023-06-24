package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class WristRunBrake extends CommandBase
{

  private final Wrist   m_wrist;
  private final Boolean m_brake;
  private int           m_loopCounter;

  public WristRunBrake(Wrist wrist, boolean brake)
  {
    m_wrist = wrist;
    m_brake = brake;

    setName("WristRunBrake");
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_loopCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_wrist.setMotorOutput(m_brake ? -0.25 : 0.25);
    m_loopCounter++;
    DataLogManager.log("motor set to down-power: " + m_brake);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return (m_loopCounter > 50 ? true : false);
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
