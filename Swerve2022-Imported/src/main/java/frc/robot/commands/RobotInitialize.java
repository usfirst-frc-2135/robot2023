
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 *
 */
public class RobotInitialize extends CommandBase
{
  public RobotInitialize( )
  {
    setName("RobotInitialize");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    RobotContainer robotContainer = RobotContainer.getInstance( );
    robotContainer.m_led.initialize( );
    robotContainer.m_power.initialize( );
    robotContainer.m_pneumatics.initialize( );
    robotContainer.m_vision.initialize( );

    robotContainer.m_swerve.initialize( );
    robotContainer.m_intake.initialize( );
    robotContainer.m_floorConveyor.initialize( );
    robotContainer.m_towerConveyor.initialize( );
    robotContainer.m_shooter.initialize( );
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
    return true;
  }
}
