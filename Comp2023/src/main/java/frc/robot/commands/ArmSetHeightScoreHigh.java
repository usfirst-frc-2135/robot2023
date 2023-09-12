
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ELConsts;
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
        new PrintCommand(getName() + ": Moving Elbow"),
        new ElbowMoveToPosition(elbow,  ELConsts.kAngleScoreHigh).asProxy(),

        new PrintCommand(getName() + ": Moving Wrist"),
        new WristMoveToAngle(wrist, WristAngle.WRIST_HIGH).asProxy(),
        
        new PrintCommand(getName() + ": Moving Extension"),
        new ExtensionMoveToLength(extension, ExtensionLength.EXTENSION_HIGH).asProxy()
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
