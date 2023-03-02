
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class PreloadHigh extends SequentialCommandGroup
{
  public PreloadHigh(Gripper gripper, Wrist wrist, Elbow elbow)
  {
    setName("PreloadHigh");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("MOVE ARM"),
        new ParallelDeadlineGroup(
          new ArmSetHeightScoreHigh(elbow, wrist)
          )
        //TODO: ADD COMMAND TO SCORE GAME PIECE - EXPEL WITH GRIPPER (?)
        /* 
        new PrintCommand("AUTO: Run second path"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoDrivePath ( swerve, "driveFromGamePiece", false)
        ),
        */
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
