
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/**
 *
 */
public class IntakeDeploy extends CommandBase
{
  private final Intake  m_intake;
  private final boolean m_intakeExtend;

  public IntakeDeploy(Intake intake, boolean intakeExtend)
  {
    m_intake = intake;
    m_intakeExtend = intakeExtend;

    setName("IntakeDeploy");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_intake.setArmSolenoid(m_intakeExtend);
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
