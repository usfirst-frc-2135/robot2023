
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.subsystems.LED;

/**
 *
 */
public class LEDSet extends CommandBase
{
  private final LEDColor m_color;
  private final LED      m_led;

  public LEDSet(LED led, LEDColor color)
  {
    m_led = led;
    m_color = color;

    setName("LEDSet");
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_led.setColor(m_color);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {

  }

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
