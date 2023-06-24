
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class AutoPathSequence extends SequentialCommandGroup
{
  public AutoPathSequence(Swerve swerve, String pathname1, PathPlannerTrajectory trajectory1, String pathname2,
      PathPlannerTrajectory trajectory2)
  {
    setName("AutoPathSequence");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Drive first path"),
        new AutoDrivePath (swerve, pathname1, trajectory1, true),

        new PrintCommand(getName() + ": Drive second path"),
        new AutoDrivePath ( swerve, pathname2, trajectory2, false),

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
