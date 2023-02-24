
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ELConsts.ElbowAngle;
import frc.robot.Constants.WRConsts.WristAngle;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class ArmSetHeightScoreMid extends SequentialCommandGroup
{
  public ArmSetHeightScoreMid(Elbow elbow, Wrist wrist)
  {
    setName("ArmSetHeightScoreMid");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("Moving Elbow"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(elbow::moveElbowDistanceIsFinished),
          new ElbowMoveToAngle(elbow, ElbowAngle.ELBOW_MID)
        ),

        new PrintCommand("Moving Wrist"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(wrist::moveWristDistanceIsFinished),
          new WristMoveToAngle(wrist, WristAngle.WRIST_MID)
        )
        //TODO: EXTEND ARM
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
