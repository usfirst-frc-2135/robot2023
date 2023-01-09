
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
public class AutoShootDriveShoot extends SequentialCommandGroup
{
  public AutoShootDriveShoot(Swerve swerve, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {

    setName("AutoShootDriveShoot");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("AUTO SHOOT DRIVE SHOOT: Use programmable delay from dashboard before starting"),
        new ParallelDeadlineGroup(
          new AutoWait(AutoTimer.TIMER1), 
          new AutoStop(swerve)
        ),
        
        new PrintCommand("AUTO: Deploy intake"), 
        new ParallelDeadlineGroup(
          new IntakeDeploy(intake, true), 
          new AutoStop(swerve)
        ),

        new PrintCommand("AUTO: Run path to a shooitng position"),
        new ParallelCommandGroup(
          new ParallelDeadlineGroup(
              new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
              new AutoDrivePath(swerve, AUTOConstants.kShootDriveShoot_path1, true) 
          ),
          new ScoringPrime(shooter, vision) 
        ),

        new PrintCommand("AUTO: shoot preloaded ball"),
        new ParallelDeadlineGroup( 
          new ScoringActionUpperHub(intake, fConv, tConv, shooter, 1), 
          new AutoStop(swerve) 
        ),
        
        new PrintCommand("AUTO: Drive to 2nd ball and intake"),
        new ParallelDeadlineGroup(
          new ParallelDeadlineGroup(
              new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
              new AutoDrivePath(swerve, AUTOConstants.kShootDriveShoot_path2, false) 
          ),
          new IntakingAction(intake, fConv, tConv),
          new ScoringPrime(shooter, vision) 
        ),

        new PrintCommand("AUTO: Run third path"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoDrivePath(swerve, AUTOConstants.kShootDriveShoot_path3, false) 
        ),

        new PrintCommand("AUTO: Shoot second ball"),
        new ParallelDeadlineGroup( 
          new ScoringActionUpperHub(intake, fConv, tConv, shooter, 2), 
          new AutoStop(swerve) 
        ),

        new PrintCommand("AUTO: Stow intake"),
        new ParallelDeadlineGroup( 
          new IntakeDeploy(intake, false), 
          new AutoStop(swerve) 
        ),

        new PrintCommand("AUTO: Use programmable delay from dashboard before starting"),
        new ParallelDeadlineGroup(
          new WaitCommand(1.0), 
          new AutoStop(swerve)
        ),
 
        new PrintCommand("AUTO: Run fourth path"),
        new ParallelCommandGroup(
          new ParallelDeadlineGroup(
            new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
            new AutoDrivePath(swerve, AUTOConstants.kShootDriveShoot_path4, false) 
          ),
          new ScoringStop(intake, fConv, tConv, shooter, vision) 
        ),

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
