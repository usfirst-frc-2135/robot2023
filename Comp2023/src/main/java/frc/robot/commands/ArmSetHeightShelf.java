
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

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
public class ArmSetHeightShelf extends SequentialCommandGroup
{
  public ArmSetHeightShelf(Elbow elbow, Extension extension, Wrist wrist)
  {
    setName("ArmSetHeightShelf");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Moving Elbow"),
        new ElbowMoveToAngle(elbow,  ElbowAngle.ELBOW_SHELF),

        new PrintCommand(getName() + ": Moving Wrist"),
        new WristMoveToAngle(wrist, WristAngle.WRIST_SHELF),
        
        new PrintCommand(getName() + ": Moving Extension"),
        new ExtensionMoveToLength(extension, ExtensionLength.EXTENSION_SHELF)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}