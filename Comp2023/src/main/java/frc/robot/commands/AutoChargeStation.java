
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
public class AutoChargeStation extends SequentialCommandGroup
{
  public AutoChargeStation(Swerve swerve)
  {
    setName("AutoChargeStation");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("AUTO PATH SEQUENCE: go to ChargeStation"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoDrivePath (swerve, "driveToChargeStation", true)
        ),

        new PrintCommand("AUTO: Balance on ChargeStation"),
        new DriveBalance(swerve)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
