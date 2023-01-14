
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
    RobotContainer rc = RobotContainer.getInstance( );
    rc.m_led.initialize( );
    rc.m_power.initialize( );
    rc.m_pneumatics.initialize( );
    rc.m_vision.initialize( );

    rc.m_swerve.initialize( );
    rc.m_intake.initialize( );
    rc.m_floorConveyor.initialize( );
    rc.m_towerConveyor.initialize( );
    rc.m_shooter.initialize( );
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
