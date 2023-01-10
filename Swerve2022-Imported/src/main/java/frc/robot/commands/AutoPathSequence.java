
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AUTOConstants;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class AutoPathSequence extends SequentialCommandGroup
{
  public AutoPathSequence(Swerve swerve)
  {
    setName("AutoPathSequence");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("AUTO PATH SEQUENCE: Run first path"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoDrivePath (swerve, AUTOConstants.path1, true)
        ),

        new PrintCommand("AUTO: Run second path"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoDrivePath ( swerve, AUTOConstants.path2, false)
        ),

        new PrintCommand("AUTO: Hold in place"),
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
