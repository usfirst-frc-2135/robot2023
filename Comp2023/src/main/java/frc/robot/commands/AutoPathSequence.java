
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class AutoPathSequence extends SequentialCommandGroup
{
  public AutoPathSequence(Swerve swerve, String pathname, String pathname2)
  {
    setName("AutoPathSequence");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": AUTO PATH SEQUENCE: Run first path"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoDrivePath (swerve, pathname, true)
        ),

        new PrintCommand(getName() + ": AUTO: Run second path"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoDrivePath ( swerve, pathname2, false)
        ),

        new PrintCommand(getName() + ": AUTO: Hold in place"),
        new AutoStop(swerve)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
