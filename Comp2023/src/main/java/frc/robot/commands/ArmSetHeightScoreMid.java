
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ELConsts.ElbowPosition;
import frc.robot.Constants.EXConsts.ExtensionLength;
import frc.robot.Constants.WRConsts.WristAngle;
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
        new ExtensionMoveToLength(extension, ExtensionLength.EXTENSION_IDLE).asProxy(),

        new ConditionalCommand(
          new SequentialCommandGroup(
            new PrintCommand(getName() + ": Move Wrist"),
            new WristMoveToAngle(wrist, WristAngle.WRIST_MID).asProxy(),

            new PrintCommand(getName() + ": Move Elbow"),
            new ElbowMoveToPosition(elbow,  ElbowPosition.ELBOW_MID).asProxy()
          ),

          new SequentialCommandGroup(
            new PrintCommand(getName() + ": Move Wrist"),
            new WristMoveToAngle(wrist, WristAngle.WRIST_MID).asProxy(),
           
            new PrintCommand(getName() + ": Move Elbow"),
            new ElbowMoveToPosition(elbow,  ElbowPosition.ELBOW_MID).asProxy()
          ),
          elbow::isElbowBelowMid
        ),

        new PrintCommand(getName() + ": Extend Extension"),
        new ExtensionMoveToLength(extension, ExtensionLength.EXTENSION_MID).asProxy()
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
