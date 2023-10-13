
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

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
public class ArmSetHeightShelf extends SequentialCommandGroup
{
  public ArmSetHeightShelf(Elbow elbow, Extension extension, Wrist wrist)
  {
    setName("ArmSetHeightShelf");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Moving Elbow"),
        new ElbowMoveToPosition(elbow,  ELConsts.kAngleSubstation).asProxy(),

        new PrintCommand(getName() + ": Moving Wrist"),
        new WristMoveToPosition(wrist, WRConsts.kAngleSubstation).asProxy(),
        
        new PrintCommand(getName() + ": Moving Extension"),
        new ExtensionMoveToPosition(extension, elbow, EXConsts.kLengthSubstation).asProxy()
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
