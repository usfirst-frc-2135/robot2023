
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ELConsts.ElbowAngle;
import frc.robot.Constants.EXConsts.ExtensionLength;
import frc.robot.Constants.GRConsts;
import frc.robot.Constants.GRConsts.GRMode;
import frc.robot.Constants.WRConsts.WristAngle;
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
        new ParallelDeadlineGroup(
          new WaitUntilCommand(wrist::moveWristAngleIsFinished),
          new WristMoveToAngle(wrist, WristAngle.WRIST_STOW)
        ),
        new PrintCommand(getName() + ": Holding Game Piece"), 
        new ParallelDeadlineGroup( 
        new GripperRun(gripper, GRMode.GR_ACQUIRE).withTimeout(0.5)
        ),
        new GripperRun(gripper, GRMode.GR_HOLD),  

        new PrintCommand(getName() + ": Move Elbow for Preload"),   
        new ParallelDeadlineGroup(
            new WaitUntilCommand(elbow::moveElbowAngleIsFinished),
            new ElbowMoveToAngle(elbow, ElbowAngle.ELBOW_HIGH)
        ),
        new PrintCommand(getName() + ": Move Extension for Preload"),   
        new ParallelDeadlineGroup(
            new WaitUntilCommand(extension::moveExtensionLengthIsFinished),
            new ExtensionMoveToLength(extension, ExtensionLength.EXTENSION_HIGH)
        ),
        new PrintCommand(getName() + ": Move Wrist for Preload"),   
        new ParallelDeadlineGroup(
            new WaitUntilCommand(wrist::moveWristAngleIsFinished),
            new WristMoveToAngle(wrist, WristAngle.WRIST_SCORE)
        ),
        new PrintCommand(getName() + ": AUTO: Gripper Score"),
        new ParallelDeadlineGroup( 
        new GripperRun(gripper, GRConsts.GRMode.GR_EXPEL).withTimeout(1.5)
        ),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(extension::moveExtensionLengthIsFinished), 
          new ExtensionMoveToLength(extension, ExtensionLength.EXTENSION_STOW)
        ),
        new GripperRun(gripper, GRConsts.GRMode.GR_STOP),
        new PrintCommand(getName() + ": AUTO: Move Arm Down"),
        new ParallelDeadlineGroup(
          new ArmSetHeightIdle(elbow, extension, wrist)
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
