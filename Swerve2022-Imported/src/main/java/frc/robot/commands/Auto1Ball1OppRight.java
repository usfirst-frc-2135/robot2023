
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
public class Auto1Ball1OppRight extends SequentialCommandGroup
{
  private boolean autoSelector( )
  {
    return SmartDashboard.getBoolean("AUTO_ShootOppBall", false);
  }

  public Auto1Ball1OppRight(Swerve swerve, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {
    setName("Auto1Ball1OppRight");

    // get values as string part of code

    addCommands(
        // Add Commands here:

        // @formatter:off   
        new PrintCommand("AUTO 1 BALL 1 OPP RIGHT: Use programmable delay from dashboard"),
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
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoDrivePath(swerve, AUTOConstants.k1Ball1OppRight_path1, true), 
          new ScoringPrime(shooter, vision)
        ),

        new PrintCommand("AUTO: shoot preloaded ball"),
        new ParallelDeadlineGroup(
          new ScoringActionUpperHub(intake, fConv, tConv, shooter, 1), 
          new AutoStop(swerve)
        ),

        new PrintCommand("AUTO: delay before next phase"), 
        new ParallelDeadlineGroup(
          new AutoWait(AutoTimer.TIMER2), 
          new AutoStop(swerve)
        ),

        new PrintCommand("drive to opponent's ball and intake"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(swerve::driveWithPathFollowerIsFinished),
          new AutoDrivePath(swerve, AUTOConstants.k1Ball1OppRight_path2, false), 
          new IntakingAction(intake, fConv, tConv),
          new ScoringPrime(shooter, vision)
        ),

        new PrintCommand("AUTO: Stow intake"), 
        new ParallelDeadlineGroup(
          new IntakeDeploy(intake, false), 
          new AutoStop(swerve)
        ),

        new PrintCommand("AUTO: Shoot opponents ball, if selected"),
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
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
