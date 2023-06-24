
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class AutoEngageChargeStation extends SequentialCommandGroup
{
  public AutoEngageChargeStation(Swerve swerve, String pathName, PathPlannerTrajectory trajectory)
  {
    setName("AutoEngageChargeStation");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Drive to ChargeStation"),
        new AutoDrivePath (swerve, pathName, trajectory, true),

        new PrintCommand(getName() + ": Balance on ChargeStation"),
        new AutoDriveBalance(swerve).withTimeout(9.0)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
