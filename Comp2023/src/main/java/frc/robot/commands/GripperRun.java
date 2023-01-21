// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GRConsts.GRMode;
import frc.robot.subsystems.Gripper;

/** An example command that uses an example subsystem. */
public class GripperRun extends CommandBase
{
  private final Gripper m_Gripper;
  private final GRMode  m_Mode;

  public GripperRun(Gripper gripper, GRMode mode)
  {
    m_Gripper = gripper;
    m_Mode = mode;

    setName("GripperRun");
    addRequirements(m_Gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_Gripper.setGripperSpeed(m_Mode);
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
