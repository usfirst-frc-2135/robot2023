
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class DriveLimelightStop extends SequentialCommandGroup
{
  public DriveLimelightStop(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {
    setName("DriveLimelightStop");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("Limelight drive stop"), 
        new ParallelDeadlineGroup(
            new ScoringStop(intake, fConv, tConv, shooter, vision), 
            new AutoStop(drivetrain)
        )
        // @formatter:off
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
