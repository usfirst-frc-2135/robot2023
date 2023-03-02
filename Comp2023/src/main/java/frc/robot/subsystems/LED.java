//
// LED subystem - LED feedback on robot
//
package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.Ports;

//
// LED subsystem class
//
public class LED extends SubsystemBase
{
  // Member objects
  private final CANdle                    m_candle        = new CANdle(Ports.kCANID_CANdle);

  private final SendableChooser<LEDColor> m_ledChooser    = new SendableChooser<LEDColor>( );
  private LEDColor                        m_previousColor = LEDColor.LEDCOLOR_OFF;

  // Constructor
  public LED( )
  {
    setName("LED");
    setSubsystem("LED");

    setColor(LEDColor.LEDCOLOR_BLUE);
    m_candle.configBrightnessScalar(0.7);

    // Add options for colors in SmartDashboard
    m_ledChooser.setDefaultOption("LED_Off", LEDColor.LEDCOLOR_OFF);
    m_ledChooser.addOption("LED_White", LEDColor.LEDCOLOR_WHITE);
    m_ledChooser.addOption("LED_Red", LEDColor.LEDCOLOR_RED);
    m_ledChooser.addOption("LED_Orange", LEDColor.LEDCOLOR_ORANGE);
    m_ledChooser.addOption("LED_Yellow", LEDColor.LEDCOLOR_YELLOW);
    m_ledChooser.addOption("LED_Green", LEDColor.LEDCOLOR_GREEN);
    m_ledChooser.addOption("LED_Blue", LEDColor.LEDCOLOR_BLUE);
    m_ledChooser.addOption("LED_Purple", LEDColor.LEDCOLOR_PURPLE);

    SmartDashboard.putData("LED_Color", m_ledChooser);
    SmartDashboard.putBoolean("LED_normalMode", false);

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run when in simulation
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {
    DataLogManager.log(getSubsystem( ) + ": Subsystem initialized!");
    setColor(LEDColor.LEDCOLOR_OFF);
  }

  public void faultDump( )
  {}

  public void setColor(LEDColor color)
  {
    final String strName;

    if (m_previousColor != color)
    {
      if (color == (LEDColor.LEDCOLOR_DASH))
        color = m_ledChooser.getSelected( );

      switch (color)
      {
        default :
        case LEDCOLOR_OFF :
          strName = "OFF";
          m_candle.setLEDs(0, 0, 0); // black
          break;
        case LEDCOLOR_WHITE :
          strName = "WHITE";
          m_candle.setLEDs(255, 255, 255); // white
          break;
        case LEDCOLOR_RED :
          strName = "RED";
          m_candle.setLEDs(255, 0, 0); // red
          break;
        case LEDCOLOR_ORANGE :
          strName = "ORANGE";
          m_candle.setLEDs(255, 80, 0); // orange
          break;
        case LEDCOLOR_YELLOW :
          strName = "YELLOW";
          m_candle.setLEDs(255, 255, 0); // yellow
          break;
        case LEDCOLOR_GREEN :
          strName = "GREEN";
          m_candle.setLEDs(0, 255, 0); // green
          break;
        case LEDCOLOR_BLUE :
          strName = "BLUE";
          m_candle.setLEDs(0, 0, 255); // blue
          break;
        case LEDCOLOR_PURPLE :
          strName = "PURPLE";
          m_candle.setLEDs(255, 0, 255); // purple
          break;
      }

      DataLogManager.log(getSubsystem( ) + ": color is now " + strName);
      m_previousColor = color;
    }
  }

  public void setNormalColor(LEDColor color)
  {
    Boolean normalMode = SmartDashboard.getBoolean("LED_normalMode", false);

    if (normalMode)
      setColor(color);
  }

  public void setLLColor(LEDColor color)
  {
    Boolean normalMode = SmartDashboard.getBoolean("LED_normalMode", false);

    if (!normalMode)
      setColor(color);
  }
}
