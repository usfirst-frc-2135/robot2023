
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ELConsts.ElbowAngle;
import frc.robot.Constants.EXConsts.ExtensionLength;
import frc.robot.Constants.WRConsts.WristAngle;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class ArmSetHeightScoreLow extends SequentialCommandGroup
{
  public ArmSetHeightScoreLow(Elbow elbow, Extension extension, Wrist wrist)
  {
    setName("ArmSetHeightScoreLow");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Retract Extension"),
        new ExtensionMoveToLength(extension, ExtensionLength.EXTENSION_IDLE).asProxy(),

        new ConditionalCommand(
          new SequentialCommandGroup(
            new PrintCommand(getName() + ": Move Elbow"),
            new ElbowMoveToAngle(elbow,  ElbowAngle.ELBOW_LOW).asProxy(),

            new PrintCommand(getName() + ": Move Wrist"),
            new WristMoveToAngle(wrist, WristAngle.WRIST_LOW).asProxy()
          ),

          new SequentialCommandGroup(
            new PrintCommand(getName() + ": Move Wrist"),
            new WristMoveToAngle(wrist, WristAngle.WRIST_LOW).asProxy(),

            new PrintCommand(getName() + ": Move Elbow"),
            new ElbowMoveToAngle(elbow,  ElbowAngle.ELBOW_LOW).asProxy()
          ),

          elbow::isElbowBelowLow
        ),

        new PrintCommand(getName() + ": Extend Extension"),
        new ExtensionMoveToLength(extension, ExtensionLength.EXTENSION_LOW).asProxy()
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
