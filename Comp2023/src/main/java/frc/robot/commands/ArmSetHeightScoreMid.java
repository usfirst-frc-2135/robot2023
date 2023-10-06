
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.EXConsts;
import frc.robot.Constants.WRConsts;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class ArmSetHeightScoreMid extends SequentialCommandGroup
{
  public ArmSetHeightScoreMid(Elbow elbow, Extension extension, Wrist wrist)
  {
    setName("ArmSetHeightScoreMid");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName()+": Retract Extension"),
        new ExtensionMoveToPosition(extension, EXConsts.kLengthIdle).asProxy(),

        new ConditionalCommand(
          new SequentialCommandGroup(
            new PrintCommand(getName() + ": Move Wrist"),
            new WristMoveToPosition(wrist, WRConsts.kAngleScoreMid).asProxy(),

            new PrintCommand(getName() + ": Move Elbow"),
            new ElbowMoveToPosition(elbow,  ELConsts.kAngleScoreMid).asProxy()
          ),

          new SequentialCommandGroup(
            new PrintCommand(getName() + ": Move Wrist"),
            new WristMoveToPosition(wrist, WRConsts.kAngleScoreMid).asProxy(),
           
            new PrintCommand(getName() + ": Move Elbow"),
            new ElbowMoveToPosition(elbow,  ELConsts.kAngleScoreMid).asProxy()
          ),
          elbow::isBelowMid
        ),

        new PrintCommand(getName() + ": Extend Extension"),
        new ExtensionMoveToPosition(extension, EXConsts.kLengthScoreMid).asProxy()
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
