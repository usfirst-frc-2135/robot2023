
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AUTOConstants.AutoTimer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class AutoShootLowHub extends SequentialCommandGroup
{
  public AutoShootLowHub(Swerve swerve, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter, Vision vision)
  {
    setName("AutoShootLowHub");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("AUTO SHOOT LOW HUB: Use programmable delay from dashboard before starting"),
        new ParallelDeadlineGroup(
          new AutoWait(AutoTimer.TIMER1), 
          new AutoStop(swerve)
        ),
                
        new PrintCommand("AUTO: Shoot into lower hub"), 
        new ParallelDeadlineGroup(
          new IntakeDeploy(intake, true), 
          new AutoShoot(swerve, intake, fConv, tConv, shooter, vision)
        ),

        new ParallelDeadlineGroup(
          new ScoringActionLowerHub(intake, fConv,tConv,shooter, 2.0), 
          new AutoStop(swerve)
        ),

        new ScoringStop(intake, fConv, tConv, shooter, vision),

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
