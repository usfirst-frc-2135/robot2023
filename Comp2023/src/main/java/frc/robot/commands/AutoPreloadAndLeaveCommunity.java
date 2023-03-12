
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.GRConsts.GRMode;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class AutoPreloadAndLeaveCommunity extends SequentialCommandGroup
{
  public AutoPreloadAndLeaveCommunity(Swerve swerve, Elbow elbow, Extension extension, Wrist wrist, Gripper gripper)
  {
    setName("AutoPreloadAndLeaveCommunity");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": AUTO: Score Preload"),        
        new ParallelDeadlineGroup(
          new ExtensionCalibrate(extension), 
          new GripperRun(gripper, GRMode.GR_EXPEL)
        ),
        new GripperRun(gripper, GRMode.GR_STOP),

        new PrintCommand(getName() + ": AUTO: Drive Off Community"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoDrivePath (swerve, "driveOutOfCommunity", true)
        ),
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
