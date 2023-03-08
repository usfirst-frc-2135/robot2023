
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
public class AutoEngageChargeStation extends SequentialCommandGroup
{
  public AutoEngageChargeStation(Swerve swerve)
  {
    setName("AutoEngageChargeStation");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": AUTO PATH SEQUENCE: go to ChargeStation"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoDrivePath (swerve, "driveToChargeStation", true)
        ),

        new PrintCommand(getName() + ": AUTO: Balance on ChargeStation"),
        new AutoDriveBalance(swerve)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
