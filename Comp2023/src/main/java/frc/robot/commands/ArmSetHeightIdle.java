
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
public class ArmSetHeightIdle extends SequentialCommandGroup
{
  public ArmSetHeightIdle(Elbow elbow, Extension extension, Wrist wrist)
  {
    setName("ArmSetHeightIdle");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName()+": Retract Extension"),
        new ExtensionMoveToLength(extension, ExtensionLength.EXTENSION_STOW),
        new ConditionalCommand(
          new SequentialCommandGroup(
            new PrintCommand(getName() + ": Move Elbow"),
            new ElbowMoveToAngle(elbow,  ElbowAngle.ELBOW_IDLE),

            new PrintCommand(getName() + ": Move Wrist"),
            new WristMoveToAngle(wrist, WristAngle.WRIST_IDLE)
          ),

          new SequentialCommandGroup(
            new PrintCommand(getName() + ": Move Wrist"),
            new WristMoveToAngle(wrist, WristAngle.WRIST_IDLE),

            new PrintCommand(getName() + ": Move Elbow"),
            new ElbowMoveToAngle(elbow,  ElbowAngle.ELBOW_IDLE)
          ),

          elbow::isElbowBelowIdle
        ),

        new PrintCommand(getName() + ": Extend Extension"),
        new ExtensionMoveToLength(extension, ExtensionLength.EXTENSION_IDLE)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
