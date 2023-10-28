
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class AutoPreloadAndStop extends SequentialCommandGroup
{
  public AutoPreloadAndStop(Swerve swerve, Elbow elbow, Extension extension, Wrist wrist, Gripper gripper)
  {
    setName("AutoPreloadAndStop");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": AUTO: Score Preload"),        
        new ExtensionCalibrate(extension).asProxy(), 
        new AutoPreloadMid(elbow, extension, wrist, gripper),

        new PrintCommand(getName() + ": AUTO: Hold in place"),
        new AutoStop(swerve)
        // @formatter:on
    );

  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
