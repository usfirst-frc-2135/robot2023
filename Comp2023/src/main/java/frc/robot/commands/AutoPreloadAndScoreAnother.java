
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        // TODO: write this command

        // @formatter:off
        new PrintCommand(getName() + ": Score Preload"),
        /* DRIVE BACKWARD + PRELOAD PATHS / COMMANDS
        */
        new AutoDrivePath (swerve, pathName1, trajectory1, true),

        // ADD COMMAND TO PICK UP GAME PIECE
        new PrintCommand(getName() + ": Drive second path"),
        new AutoDrivePath ( swerve, pathName2, trajectory2, false),

        // ADD COMMAND TO SCORE GAME PIECE

        new PrintCommand(getName() + ": Hold in place"),
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
