
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class DriveLimelightShoot extends SequentialCommandGroup
{
  public DriveLimelightShoot(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {
    setName("DriveLimelightShoot");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("Limelight drive and then shoot"), 
        new ParallelCommandGroup(
            new DriveLimelight(drivetrain, vision, false),
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitUntilCommand(drivetrain::driveWithLimelightIsFinished),
                    new ScoringPrime(shooter, vision)
                ), 
                new ScoringActionUpperHub(intake, fConv, tConv, shooter, 120.0)
            )
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
