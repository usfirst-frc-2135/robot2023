
// ROBOTBUILDER TYPE: WaitCommand.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 *
 */
public class AutoWait extends WaitCommand
{
    private double m_timeout;

    public AutoWait(double timeout)
    {
        super(timeout);
        m_timeout = timeout;

        // m_subsystem = subsystem;
        // addRequirements(m_subsystem);
    }

    // Called just before this Command runs the first time
    @Override public void initialize()
    {
        super.initialize();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override public void execute() {}

    // Called once after isFinished returns true
    @Override public void end(boolean interrupted)
    {
        super.end(interrupted);
    }

    @Override public boolean runsWhenDisabled()
    {
        return false;
    }
}
