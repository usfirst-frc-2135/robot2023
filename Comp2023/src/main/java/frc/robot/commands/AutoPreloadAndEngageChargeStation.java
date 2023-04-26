
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.GRConsts.GRMode;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class AutoPreloadAndEngageChargeStation extends SequentialCommandGroup
{
  public AutoPreloadAndEngageChargeStation(Swerve swerve, Elbow elbow, Extension extension, Wrist wrist, Gripper gripper,
      String pathName, PathPlannerTrajectory trajectory)
  {
    setName("AutoPreloadAndEngageChargeStation");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Score Preload"),        
        new ExtensionCalibrate(extension).asProxy(), 
        new ArmSetHeightIdle(elbow, extension, wrist),
        new GripperRun(gripper, GRMode.GR_EXPEL),
        new WaitCommand(0.25),

        new GripperRun(gripper, GRMode.GR_STOP),
        
        new PrintCommand(getName() + ": Drive to ChargeStation"),
        new AutoEngageChargeStation(swerve, pathName, trajectory),

        new PrintCommand(getName() + ": Hold in place"),
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
