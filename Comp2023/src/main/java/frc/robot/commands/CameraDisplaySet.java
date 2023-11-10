//
// LED Set command - sets CANdle to desired mode
//
package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class CameraDisplaySet extends CommandBase
{
  private int          m_stream;
  private NetworkTable m_table;            // Network table reference for getting LL values
  private Vision       m_vision;

  public CameraDisplaySet(Vision vision, int stream)
  {
    m_table = NetworkTableInstance.getDefault( ).getTable("limelight");
    m_vision = vision;
    m_stream = stream;

    setName("CameraDisplaySet");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_vision.setCameraDisplay(m_stream);
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
