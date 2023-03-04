
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class AutoPreloadAndEngageChargeStation extends SequentialCommandGroup
{
  public AutoPreloadAndEngageChargeStation(Swerve swerve, Elbow elbow, Extension extension, Wrist wrist, Gripper gripper)
  {
    setName("AutoPreloadAndEngageChargeStation");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": AUTO: Preload"),        
        new ParallelDeadlineGroup(
            new AutoPreloadHigh(elbow, extension, wrist, gripper)
        ),
        new PrintCommand(getName() + ": AUTO: Run to ChargeStation"),
        new ParallelDeadlineGroup(
          new AutoChargeStation(swerve)
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
