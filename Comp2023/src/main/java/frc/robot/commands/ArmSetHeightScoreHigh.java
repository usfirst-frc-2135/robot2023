
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ELConsts.ElbowAngle;
import frc.robot.Constants.EXConsts.ExtensionLength;
import frc.robot.Constants.WRConsts.WristAngle;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class ArmSetHeightScoreHigh extends SequentialCommandGroup
{
  public ArmSetHeightScoreHigh(Elbow elbow, Extension extension, Wrist wrist)
  {
    setName("ArmSetHeightScoreHigh");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Move Wrist"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(wrist::moveWristAngleIsFinished),
          new WristMoveToAngle(wrist, WristAngle.WRIST_STOW)
        ),
        new PrintCommand(getName() + ": Moving Elbow"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(elbow::moveElbowAngleIsFinished),
          new ElbowMoveToAngle(elbow, ElbowAngle.ELBOW_HIGH)
        ),
        new PrintCommand(getName() + ": Moving Extension"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(extension::moveExtensionLengthIsFinished),
          new ExtensionMoveToLength(extension, ExtensionLength.EXTENSION_HIGH)
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
