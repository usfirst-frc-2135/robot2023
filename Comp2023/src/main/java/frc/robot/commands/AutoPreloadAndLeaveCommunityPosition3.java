
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.GRConsts;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class AutoPreloadAndLeaveCommunityPosition3 extends SequentialCommandGroup
{
  public AutoPreloadAndLeaveCommunityPosition3(Swerve swerve, Elbow elbow, Extension extension, Wrist wrist, Gripper gripper)
  {
    setName("AutoPreloadAndLeaveCommunityPosition3");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("AUTO: Score Preload"),        
        new ParallelDeadlineGroup(
            new ArmSetHeightScoreHigh(elbow, extension, wrist),
            new GripperRun(gripper, GRConsts.GRMode.GR_EXPEL).withTimeout(1)
        ),
        new PrintCommand("AUTO: Stop Preload"),
        new ParallelDeadlineGroup(
          new GripperRun(gripper, GRConsts.GRMode.GR_STOP),
          new ArmSetHeightStow(elbow, extension, wrist)
        ),
        new ParallelDeadlineGroup(        
        new PrintCommand("AUTO: Drive Off Community"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoDrivePath ( swerve, "driveOffCommunity2", false)
        )
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
