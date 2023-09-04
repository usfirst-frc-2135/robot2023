
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

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
public class ArmSetHeightStow extends SequentialCommandGroup
{
  public ArmSetHeightStow(Elbow elbow, Extension extension, Wrist wrist)
  {
    setName("ArmSetHeightStow");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Moving Wrist"),
        new WristMoveToAngle(wrist, WristAngle.WRIST_STOW).asProxy(),

        new PrintCommand(getName() + ": Moving Extension"),
        new ExtensionMoveToLength(extension, ExtensionLength.EXTENSION_STOW).asProxy(),

        new PrintCommand(getName() + ": Moving Elbow"),
        new ElbowMoveToPosition(elbow,  ElbowPosition.ELBOW_STOW).asProxy()
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
