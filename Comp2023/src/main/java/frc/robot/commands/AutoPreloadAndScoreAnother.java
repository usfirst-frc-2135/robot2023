
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class AutoPreloadAndScoreAnother extends SequentialCommandGroup
{
  public AutoPreloadAndScoreAnother(Swerve swerve, String pathName1, PathPlannerTrajectory trajectory1, String pathName2,
      PathPlannerTrajectory trajectory2)
  {
    setName("AutoPreloadAndScoreAnother");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": AUTO PATH SEQUENCE: Run first path"),
        /* TODO: DRIVE BACKWARD + PRELOAD PATHS / COMMANDS
        */
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoDrivePath (swerve, pathName1, trajectory1, true)
        ),
        //TODO: ADD COMMAND TO PICK UP GAME PIECE
        new PrintCommand(getName() + ": AUTO: Run second path"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoDrivePath ( swerve, pathName2, trajectory2, false)
        ),
        //TODO: ADD COMMAND TO SCORE GAME PIECE

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
