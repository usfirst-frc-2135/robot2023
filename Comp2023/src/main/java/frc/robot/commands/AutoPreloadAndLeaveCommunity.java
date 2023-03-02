
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
public class AutoPreloadAndLeaveCommunity extends SequentialCommandGroup
{
  public AutoPreloadAndLeaveCommunity(Swerve swerve)
  {
    setName("AutoPreloadAndLeaveCommunity");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": AUTO PATH SEQUENCE: Run first path"),
        /* TODO: DRIVE BACKWARD + PRELOAD PATHS / COMMANDS
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoDrivePath (swerve, "driveOffCommunity", true)
        ),
        */
        new PrintCommand(getName() + ": AUTO: Run second path"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoEngageChargeStation(swerve)
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
