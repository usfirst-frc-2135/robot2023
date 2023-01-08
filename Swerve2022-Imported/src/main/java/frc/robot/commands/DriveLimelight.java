
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VIConsts;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class DriveLimelight extends CommandBase
{
  private final Drivetrain m_drivetrain;
  private final Vision     m_vision;
  private final boolean    m_endAtTarget;

  public DriveLimelight(Drivetrain drivetrain, Vision vision, boolean endAtTarget)
  {
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_endAtTarget = endAtTarget;

    setName("DriveLimelight");
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_vision.setLEDMode(VIConsts.LED_ON);
    m_vision.setCameraDisplay(VIConsts.PIP_MAIN);
    m_drivetrain.driveWithLimelightInit(m_endAtTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_drivetrain.driveWithLimelightExecute( );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return (m_endAtTarget) ? m_drivetrain.driveWithLimelightIsFinished( ) : false;
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
