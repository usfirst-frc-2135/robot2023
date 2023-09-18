
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
public class ArmSetHeightIdle extends SequentialCommandGroup
{
  public ArmSetHeightIdle(Elbow elbow, Extension extension, Wrist wrist)
  {
    setName("ArmSetHeightIdle");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName()+": Retract Extension"),
        new ExtensionMoveToLength(extension, EXConsts.kLengthIdle).asProxy(),
        new ConditionalCommand(
          new SequentialCommandGroup(
            new PrintCommand(getName() + ": Move Elbow"),
            new ElbowMoveToPosition(elbow,  ELConsts.kAngleIdle).asProxy(),

            new PrintCommand(getName() + ": Move Wrist"),
            new WristMoveToPosition(wrist, WRConsts.kAngleIdle).asProxy()
          ),

          new SequentialCommandGroup(
            new PrintCommand(getName() + ": Move Wrist"),
            new WristMoveToPosition(wrist, WRConsts.kAngleIdle).asProxy(),

            new PrintCommand(getName() + ": Move Elbow"),
            new ElbowMoveToPosition(elbow,  ELConsts.kAngleIdle).asProxy()
          ),

          elbow::isElbowBelowIdle
        ),

        new PrintCommand(getName() + ": Extend Extension"),
        new ExtensionMoveToLength(extension, EXConsts.kLengthIdle).asProxy()
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
