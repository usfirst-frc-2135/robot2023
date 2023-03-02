
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SnapConstants;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class DriveSnap extends CommandBase
{
  private final Swerve m_swerve;
  private final double m_snapAngle;
  private final Timer  m_safetyTimer = new Timer( );

  public DriveSnap(Swerve swerve, double snapAngle)
  {
    setName("DriveSnap");

    m_swerve = swerve;
    m_snapAngle = snapAngle;

    setName("DriveSnap");
    // Does NOT require swerve, since this works together with DriveTelop
    // addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_safetyTimer.restart( );
    m_swerve.driveSnapInit(m_snapAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_safetyTimer.stop( );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    boolean timedOut = m_safetyTimer.hasElapsed(SnapConstants.kTimeout);
    if (timedOut)
      DataLogManager.log(getName( ) + ": has timed out!");
    return m_swerve.driveIsSnapFinished(timedOut);
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
