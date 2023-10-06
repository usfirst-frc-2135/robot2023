
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.EXConsts;
import frc.robot.Constants.GRConsts;
import frc.robot.Constants.GRConsts.GRMode;
import frc.robot.Constants.WRConsts;
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
        new PrintCommand(getName() + ": Making Sure Wrist is Set at Stow"),
        new WristMoveToPosition(wrist, WRConsts.kAngleStow).asProxy(),

        new PrintCommand(getName() + ": Holding Game Piece"), 
        new GripperRun(gripper, GRMode.GR_ACQUIRE),
        new WaitCommand(0.25),

        new GripperRun(gripper, GRMode.GR_HOLD),  

        new PrintCommand(getName() + ": Move Elbow for Preload"),   
        new ElbowMoveToPosition(elbow, ELConsts.kAngleScoreHigh).asProxy(),

        new PrintCommand(getName() + ": Move Extension for Preload"),   
        new ExtensionMoveToPosition(extension, EXConsts.kLengthScoreHigh).asProxy(),

        new PrintCommand(getName() + ": Move Wrist for Preload"),   
        new WristMoveToPosition(wrist, WRConsts.kAngleScore).asProxy(),

        new PrintCommand(getName() + ": AUTO: Gripper Score"),
        new GripperRun(gripper, GRConsts.GRMode.GR_EXPEL),

        new ExtensionMoveToPosition(extension, EXConsts.kLengthScoreMid).asProxy(),
        new GripperRun(gripper, GRConsts.GRMode.GR_STOP),

        new PrintCommand(getName() + ": AUTO: Move Arm Down"),
        new ArmSetHeightIdle(elbow, extension, wrist)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
