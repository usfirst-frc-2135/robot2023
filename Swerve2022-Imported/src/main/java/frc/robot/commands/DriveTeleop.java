
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class DriveTeleop extends CommandBase
{
  private final Drivetrain     m_drivetrain;
  private final XboxController m_driverPad;
  private final int            m_speedAxis;
  private final int            m_rotationAxis;

  public DriveTeleop(Drivetrain drivetrain, XboxController driverPad, int speedAxis, int rotationAxis)
  {
    m_drivetrain = drivetrain;
    m_driverPad = driverPad;
    m_speedAxis = speedAxis;
    m_rotationAxis = rotationAxis;

    setName("DriveTeleop");
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    //TODO: replace with updated method
    //m_drivetrain.driveWithJoysticksInit( );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    double rotation = m_driverPad.getRawAxis(m_rotationAxis);
    double speed = m_driverPad.getRawAxis(m_speedAxis);

    //TODO: replace with updated method
    //m_drivetrain.driveWithJoysticksExecute(speed, rotation);
    m_drivetrain.driveWithJoystick(m_driverPad, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    //TODO: replace with updated method
    //m_drivetrain.driveWithJoysticksEnd( );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return false;
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
