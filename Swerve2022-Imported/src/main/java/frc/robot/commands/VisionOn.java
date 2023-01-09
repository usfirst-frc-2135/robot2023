
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VIConsts;
import frc.robot.Constants.VIConsts.VIRequests;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class VisionOn extends CommandBase
{
  private final Vision     m_vision;
  private final VIRequests m_mode;

  public VisionOn(Vision vision, VIRequests mode)
  {
    m_vision = vision;
    m_mode = mode;

    setName("VisionOn");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {

    switch (m_mode)
    {

      default :
      case VISION_ON :
        m_vision.setLEDMode(VIConsts.LED_ON);
        m_vision.setCameraDisplay(VIConsts.PIP_MAIN);
        break;
      case VISION_OFF :
        m_vision.setLEDMode(VIConsts.LED_OFF);
        m_vision.setCameraDisplay(VIConsts.PIP_SECONDARY);
        break;
      case VISION_TOGGLE :
        if (m_vision.getLEDMode( ) == VIConsts.LED_ON)
        {
          m_vision.setLEDMode(VIConsts.LED_OFF);
          m_vision.setCameraDisplay(VIConsts.PIP_SECONDARY);
        }
        else
        {
          m_vision.setLEDMode(VIConsts.LED_ON);
          m_vision.setCameraDisplay(VIConsts.PIP_MAIN);
        }
        break;
    }
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
