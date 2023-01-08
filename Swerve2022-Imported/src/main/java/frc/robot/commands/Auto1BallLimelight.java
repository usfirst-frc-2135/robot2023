
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 *
 */
public class Auto1BallLimelight extends SequentialCommandGroup
{
    public Auto1BallLimelight()
    {
        addCommands(
            // Add Commands here:
            // Also add parallel commands using the
            //
            // addCommands(
            //      new command1(argsN, subsystem),
            //      parallel(
            //          new command2(argsN, subsystem),
            //          new command3(argsN, subsystem)
            //      )
            //  );

        );
    }

    @Override public boolean runsWhenDisabled()
    {
        return false;
    }
}
