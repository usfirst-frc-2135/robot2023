
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FCConsts.FCMode;
import frc.robot.Constants.INConsts.INMode;
import frc.robot.Constants.SHConsts.SHMode;
import frc.robot.Constants.TCConsts.TCMode;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;

/**
 *
 */
public class ScoringActionLowerHub extends SequentialCommandGroup
{
  public ScoringActionLowerHub(Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter, double waitTime)
  {
    setName("ScoringActionLowHub");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("ScoringActionLowerHub"), 
        new ParallelDeadlineGroup(
          new WaitUntilCommand(shooter::isAtDesiredSpeed), 
          new ShooterRun(shooter, SHMode.SHOOTER_LOWERHUB)
        ),
        
        new ParallelDeadlineGroup(
          new WaitCommand(waitTime),       
          new TowerConveyorRun(tConv, TCMode.TCONVEYOR_ACQUIRE),
          new FloorConveyorRun(fConv, FCMode.FCONVEYOR_ACQUIRE),
          new IntakeRun(intake, INMode.INTAKE_ACQUIRE) 
        )
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
