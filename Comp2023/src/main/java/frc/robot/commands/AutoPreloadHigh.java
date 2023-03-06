
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GRConsts;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class AutoPreloadHigh extends SequentialCommandGroup
{
  public AutoPreloadHigh(Elbow elbow, Extension extension, Wrist wrist, Gripper gripper)
  {
    setName("AutoPreloadHigh");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": AUTO: Move Arm for Preload"),        
        new ParallelDeadlineGroup(
            new ArmSetHeightScoreHigh(elbow, extension, wrist)
        ),
        new PrintCommand(getName() + ": AUTO: Gripper Score"),
        new ParallelDeadlineGroup(
          new GripperRun(gripper, GRConsts.GRMode.GR_EXPEL).withTimeout(1)
        ),
        new ParallelDeadlineGroup(
          new GripperRun(gripper, GRConsts.GRMode.GR_STOP)
          ),

        new PrintCommand(getName() + ": AUTO: Move Arm Down"),
        new ParallelDeadlineGroup(
          new ArmSetHeightStow(elbow, extension, wrist)
        )
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
