
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AUTOConstants;
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
public class AutoShoot extends SequentialCommandGroup
{
  public AutoShoot(Swerve swerve, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter, Vision vision)
  {
    setName("AutoShoot");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("AUTO SHOOT: Use programmable delay from dashboard before starting"),
        new ParallelDeadlineGroup(
          new AutoWait(AutoTimer.TIMER1), 
          new AutoStop(swerve)
        ),
        
        new PrintCommand("AUTO: Deply intake at start"),
        new ParallelDeadlineGroup(
          new IntakeDeploy(intake, true), 
          new AutoStop(swerve)
        ),

        new PrintCommand("AUTO: Run the path and prime the shooter"),
        new ParallelCommandGroup(
          new ParallelDeadlineGroup(
            new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
            new AutoDrivePath(swerve, AUTOConstants.kShoot_path, true)
          ), 
          new ScoringPrime(shooter, vision)
        ),

        new PrintCommand("AUTO: Shoot the cargo"),
        new ParallelDeadlineGroup(
          new ScoringActionUpperHub(intake, fConv, tConv, shooter, 5.0), 
          new AutoStop(swerve)
        ),

        new PrintCommand("AUTO: Stop the shooter"),
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
