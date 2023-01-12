package frc.robot.team1678.frc2022.controlboard;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class ControlBoard
{
  public static final double kSwerveDeadband = Constants.kStickDeadband;

  private XboxController     m_driver;
  private XboxController     m_operator;

  private final int          kDpadUp         = 0;
  private final int          kDpadRight      = 90;
  private final int          kDpadDown       = 180;
  private final int          kDpadLeft       = 270;

  public ControlBoard(XboxController driver, XboxController operator)
  {
    m_driver = driver;
    m_operator = operator;
  }

  public enum SwerveCardinal
  {
    // @formatter:off
    NONE(0),

    FORWARDS(0), 
    LEFT(270), 
    RIGHT(90), 
    BACKWARDS(180),

    FAR_FENDER(143), 
    RIGHT_FENDER(233), 
    LEFT_FENDER(53), 
    CLOSE_FENDER(323);
    // @formatter:on

    public final double degrees;

    SwerveCardinal(double degrees)
    {
      this.degrees = degrees;
    }
  }

  public SwerveCardinal getSwerveSnap( )
  {
    // FENDER SNAPS
    if (m_driver.getAButton( ))
    {
      return SwerveCardinal.CLOSE_FENDER;
    }
    if (m_driver.getBButton( ))
    {
      return SwerveCardinal.LEFT_FENDER;
    }
    if (m_driver.getXButton( ))
    {
      return SwerveCardinal.RIGHT_FENDER;
    }
    if (m_driver.getYButton( ))
    {
      return SwerveCardinal.FAR_FENDER;
    }

    // CARDINAL SNAPS
    switch (m_driver.getPOV( ))
    {
      case kDpadUp :
        return SwerveCardinal.FORWARDS;
      case kDpadLeft :
        return SwerveCardinal.RIGHT;
      case kDpadRight :
        return SwerveCardinal.LEFT;
      case kDpadDown :
        return SwerveCardinal.BACKWARDS;
      default :
        return SwerveCardinal.NONE;
    }
  }
}
