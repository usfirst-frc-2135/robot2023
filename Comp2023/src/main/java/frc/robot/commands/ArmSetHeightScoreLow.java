
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
public class ArmSetHeightScoreLow extends SequentialCommandGroup
{
  public ArmSetHeightScoreLow(Elbow elbow, Wrist wrist)
  {
    setName("ArmSetHeightScoreLow");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("Moving Elbow"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(elbow::moveElbowAngleIsFinished),
          new ElbowMoveToAngle(elbow, ElbowAngle.ELBOW_LOW)
        ),

        new PrintCommand("Moving Wrist"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(wrist::moveWristAngleIsFinished),
          new WristMoveToAngle(wrist, WristAngle.WRIST_LOW)
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
