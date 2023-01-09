
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.INConsts.INMode;
import frc.robot.subsystems.Intake;

/**
 *
 */
public class IntakeRun extends CommandBase
{
  private final Intake m_intake;
  private final INMode m_mode;

  public IntakeRun(Intake intake, INMode mode)
  {
    m_intake = intake;
    m_mode = mode;

    setName("IntakeRun");
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_intake.setIntakeSpeed(m_mode);
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
    return true;
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
