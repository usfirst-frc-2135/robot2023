
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
public class ArmSetHeightScoreLow extends SequentialCommandGroup
{
  public ArmSetHeightScoreLow(Elbow elbow, Extension extension, Wrist wrist)
  {
    setName("ArmSetHeightScoreLow");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Retract Extension"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(extension::moveExtensionLengthIsFinished),
          new ExtensionMoveToLength(extension, ExtensionLength.EXTENSION_STOW)
        ),   
        new ConditionalCommand(
          new SequentialCommandGroup(
            new PrintCommand(getName() + ": Move Elbow"),
            new ParallelDeadlineGroup(
              new WaitUntilCommand(elbow::moveElbowAngleIsFinished),
              new ElbowMoveToAngle(elbow, ElbowAngle.ELBOW_LOW)),
            new PrintCommand(getName() + ": Move Wrist"),
            new ParallelDeadlineGroup(          
              new WaitUntilCommand(wrist::moveWristAngleIsFinished),
              new WristMoveToAngle(wrist, WristAngle.WRIST_LOW))
          ),
          new SequentialCommandGroup(
            new PrintCommand(getName() + ": Move Wrist"),
            new ParallelDeadlineGroup(          
              new WaitUntilCommand(wrist::moveWristAngleIsFinished),
              new WristMoveToAngle(wrist, WristAngle.WRIST_LOW)),
            new PrintCommand(getName() + ": Move Elbow"),
            new ParallelDeadlineGroup(
              new WaitUntilCommand(elbow::moveElbowAngleIsFinished),
              new ElbowMoveToAngle(elbow, ElbowAngle.ELBOW_LOW))
          ),
          elbow::isElbowBelowDesired
        ),

        new PrintCommand(getName() + ": Extend Extension"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(extension::moveExtensionLengthIsFinished),
          new ExtensionMoveToLength(extension, ExtensionLength.EXTENSION_LOW)
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
