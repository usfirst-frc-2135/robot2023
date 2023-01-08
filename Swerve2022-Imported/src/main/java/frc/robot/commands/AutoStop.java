
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class AutoStop extends CommandBase
{
    private final Drivetrain m_drivetrain;

    public AutoStop(Drivetrain subsystem)
    {
        m_drivetrain = subsystem;
        addRequirements(m_drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override public void execute() {}

    // Called once the command ends or is interrupted.
    @Override public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override public boolean isFinished()
    {
        return false;
    }

    @Override public boolean runsWhenDisabled()
    {
        return false;
    }
}
