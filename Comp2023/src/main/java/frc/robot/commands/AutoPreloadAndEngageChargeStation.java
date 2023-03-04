
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GRConsts;
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
  public AutoPreloadAndEngageChargeStation(Swerve swerve, Gripper gripper, Elbow elbow, Wrist wrist, Extension extension)
  {
    setName("AutoPreloadAndLeaveCommunity");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("AUTO PATH SEQUENCE: Run first path"),
        new PrintCommand("AUTO: Move Arm for Preload"),        
        new ParallelDeadlineGroup(
            new ArmSetHeightScoreHigh(elbow, extension, wrist)
        ),
        new PrintCommand("AUTO: Gripper Score"),
        new ParallelDeadlineGroup(
          new GripperRun(gripper, GRConsts.GRMode.GR_EXPEL).withTimeout(1)
        ),
        new ParallelDeadlineGroup(
          new GripperRun(gripper, GRConsts.GRMode.GR_STOP)
          ),

        new PrintCommand("AUTO: Move Arm Down"),
        new ParallelDeadlineGroup(
          new ArmSetHeightStow(elbow, extension, wrist)
        ),
        new PrintCommand("AUTO: Run to ChargeStation"),
        new ParallelDeadlineGroup(
          new AutoChargeStation(swerve)
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
