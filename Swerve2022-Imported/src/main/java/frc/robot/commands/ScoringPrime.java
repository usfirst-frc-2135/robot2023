
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SHConsts.SHMode;
import frc.robot.Constants.VIConsts.VIRequests;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class ScoringPrime extends SequentialCommandGroup
{
  public ScoringPrime(Shooter shooter, Vision vision)
  {
    setName("ScoringPrime");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("ScoringPrime"), 
        new VisionOn(vision, VIRequests.VISION_ON),
        new ShooterRun(shooter, SHMode.SHOOTER_PRIME)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
