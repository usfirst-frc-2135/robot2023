
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FCConsts.FCMode;
import frc.robot.Constants.INConsts.INMode;
import frc.robot.Constants.TCConsts.TCMode;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TowerConveyor;

/**
 *
 */
public class IntakingStop extends SequentialCommandGroup
{
  public IntakingStop(Intake intake, FloorConveyor fConv, TowerConveyor tConv)
  {
    setName("IntakingStop");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("IntakingStop"), 
        new IntakeRun(intake, INMode.INTAKE_STOP), 
        new FloorConveyorRun(fConv, FCMode.FCONVEYOR_STOP),
        new TowerConveyorRun(tConv, TCMode.TCONVEYOR_STOP)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
