
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
public class Auto1Ball2OppLeft extends SequentialCommandGroup
{
  private boolean autoSelector( )
  {
    return SmartDashboard.getBoolean("AUTO_ShootOppBall", false);
  }

  public Auto1Ball2OppLeft(Swerve swerve, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter, Vision vision)
  {
    setName("AutoBall2OppLeft");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("AUTO 1 BALL 2 OPP LEFT: Use programmable delay from dashboard"),
        new ParallelDeadlineGroup(
          new AutoWait(AutoTimer.TIMER1), 
          new AutoStop(swerve)
        ),

        new PrintCommand("AUTO: Deply intake"), 
        new ParallelDeadlineGroup(
          new IntakeDeploy(intake, true), 
          new AutoStop(swerve)
        ),

      new PrintCommand("AUTO: Run path to a shooting position"), 
      new ParallelDeadlineGroup(
        new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished), 
        new AutoDrivePath(swerve, AUTOConstants.k1Ball2OppLeft_path1, true), 
        new ScoringPrime(shooter, vision)
      ), 

      new PrintCommand("AUTO: Shoot preloaded ball"), 
      new ParallelDeadlineGroup(
        new ScoringActionUpperHub(intake, fConv, tConv, shooter, 1.0), 
        new AutoStop(swerve)
      ), 

      new PrintCommand("AUTO: Drive to 1st opponent's ball and intake"), 
      new ParallelDeadlineGroup(
        new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished), 
        new AutoDrivePath(swerve, AUTOConstants.k1Ball2OppLeft_path2, false), 
        new IntakingAction(intake, fConv, tConv)
      ), 

      new PrintCommand("AUTO: Use programmable delay from dashboard before next phase"),
      new ParallelDeadlineGroup(
        new AutoWait(AutoTimer.TIMER2), 
        new AutoStop(swerve)
      ),

      new PrintCommand("AUTO: Drive to 2nd opponent's ball and intake"), 
      new ParallelDeadlineGroup(
        new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished), 
        new AutoDrivePath(swerve, AUTOConstants.k1Ball2OppLeft_path3, false), 
        new ScoringPrime(shooter, vision), 
        new IntakingAction(intake, fConv, tConv)
      ), 

      new PrintCommand("AUTO: Stow intake"),
      new ParallelDeadlineGroup(
        new IntakeDeploy(intake, false), 
        new AutoStop(swerve)
      ),

      new PrintCommand("AUTO: Shoot low hub, if selected"),
      new ConditionalCommand(
        new ParallelDeadlineGroup(
          new ScoringActionLowerHub(intake, fConv, tConv, shooter, 2), 
          new AutoStop(swerve)
        ), 
        new AutoStop(swerve),
        this::autoSelector
      ),

      new PrintCommand("AUTO: stop shooitng and hold positin"), 
      new ParallelDeadlineGroup( 
        new ScoringStop(intake, fConv, tConv, shooter, vision), 
        new AutoStop(swerve) 
      ),
      
      new AutoStop(swerve) 
      // @formatter:on
    );
  }

  @Override
  public final boolean runsWhenDisabled( )
  {
    return false;
  }
}
