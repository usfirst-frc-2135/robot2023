//
// Gripper Run command - sets motors to desired mode
//
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GRConsts.GRMode;
import frc.robot.subsystems.Gripper;

//
// Gripper Run command
//
public class GripperRun extends CommandBase
{
  // Member variables/objects
  private final Gripper m_gripper;
  private final GRMode  m_mode;

  public GripperRun(Gripper gripper, GRMode mode)
  {
    m_gripper = gripper;
    m_mode = mode;

    setName("GripperRun");
    addRequirements(m_gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_gripper.setGripperSpeed(m_mode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return true;
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
