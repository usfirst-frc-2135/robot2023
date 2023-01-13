// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VIConsts;

/**
 *
 */
public class Vision extends SubsystemBase
{
  // Objects
  public MedianFilter  m_tyfilter    = new MedianFilter(5); // median filter y values to remove outliers (5 sample)
  public MedianFilter  m_tvfilter    = new MedianFilter(5); // median filter v values to remove outliers (5 sample)

  // Declare module variables
  private double       m_distance1   = VIConsts.kLLDistance1;   // x position in inches for first reference point
  private double       m_vertOffset1 = VIConsts.kLLVertOffset1; // y reading in degrees for first reference point
  private double       m_distance2   = VIConsts.kLLDistance2;   // x position in inches for second reference point
  private double       m_vertOffset2 = VIConsts.kLLVertOffset2; // y reading in degrees for second reference point
  private double       m_slope;   // Linear regressions slope from calibration
  private double       m_offset;  // Linear regressions slope from calibration

  private NetworkTable m_table;            // Network table reference for getting LL values

  private double       m_targetHorizAngle; // LL Target horizontal Offset from Crosshair to Target (-27 to 27 deg)
  private double       m_targetVertAngle;  // LL Target vertical Offset from Crosshair to Target (-20.5 to 20.5 deg)
  private double       m_targetArea;       // LL Target Area (0% of image to 100% of image)
  private double       m_targetSkew;       // LL Target Skew or rotation (-90 to 0 deg)
  private boolean      m_targetValid;      // LL Target Valid or not

  private double       m_distLL;           // calculated distance in inches for the current y value

  /**
   *
   */
  public Vision( )
  {
    setName("Vision");
    setSubsystem("Vision");

    // Get the Network table reference once for all methods
    m_table = NetworkTableInstance.getDefault( ).getTable("limelight");

    // Set camera and LED display
    setLEDMode(VIConsts.LED_ON);

    // Put all the needed widgets on the dashboard
    SmartDashboard.putNumber("VI_distance1", m_distance1);
    SmartDashboard.putNumber("VI_distance2", m_distance2);
    SmartDashboard.putNumber("VI_vertOffset1", m_vertOffset1);
    SmartDashboard.putNumber("VI_vertOffset2", m_vertOffset2);

    SmartDashboard.setDefaultBoolean("VI_OVERRIDE", false);
    SmartDashboard.putNumber("VI_OVERRIDE_TX", 0.0);
    SmartDashboard.putNumber("VI_OVERRIDE_TY", 0.0);
    SmartDashboard.putNumber("VI_OVERRIDE_TA", 0.0);
    SmartDashboard.putNumber("VI_OVERRIDE_TS", 0.0);
    SmartDashboard.putNumber("VI_OVERRIDE_TV", 0.0);

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    if (SmartDashboard.getBoolean("VI_OVERRIDE", false))
    {
      // Allow the limelight to be bypassed by entries from the dashboard
      m_targetHorizAngle = SmartDashboard.getNumber("VI_OVERRIDE_TX", 0.0);
      m_targetVertAngle = SmartDashboard.getNumber("VI_OVERRIDE_TY", 0.0);
      m_targetArea = SmartDashboard.getNumber("VI_OVERRIDE_TA", 0.0);
      m_targetSkew = SmartDashboard.getNumber("VI_OVERRIDE_TS", 0.0);
      m_targetValid = (SmartDashboard.getNumber("VI_OVERRIDE_TV", 0.0) > 0.5);
    }
    else
    {
      m_targetHorizAngle = m_table.getEntry("tx").getDouble(0.0);
      m_targetVertAngle = m_tyfilter.calculate(m_table.getEntry("ty").getDouble(0.0));
      m_targetArea = m_table.getEntry("ta").getDouble(0.0);
      m_targetSkew = m_table.getEntry("ts").getDouble(0.0);
      m_targetValid = m_tvfilter.calculate(m_table.getEntry("tv").getDouble(0.0)) > 0.5;
    }

    m_distLL = calculateDist(m_targetVertAngle);

    SmartDashboard.putNumber("VI_horizAngle", m_targetHorizAngle);
    SmartDashboard.putNumber("VI_vertAngle", m_targetVertAngle);
    SmartDashboard.putNumber("VI_area", m_targetArea);
    SmartDashboard.putNumber("VI_skew", m_targetSkew);
    SmartDashboard.putBoolean("VI_valid", m_targetValid);

    SmartDashboard.putNumber("VI_distLL", m_distLL);
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run when in simulation
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {
    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");

    setLEDMode(VIConsts.LED_OFF);
    setCameraDisplay(VIConsts.PIP_SECONDARY);

    syncStateFromDashboard( );
  }

  public double getHorizOffsetDeg( )
  {
    return m_targetHorizAngle;
  }

  public double getVertOffsetDeg( )
  {
    return m_targetVertAngle;
  }

  public double getTargetArea( )
  {
    return m_targetArea;
  }

  public double getTargetSkew( )
  {
    return m_targetSkew;
  }

  public boolean getTargetValid( )
  {
    return m_targetValid;
  }

  public double getDistLimelight( )
  {
    return m_distLL;
  }

  public void setLEDMode(int mode)
  {
    DataLogManager.log(getSubsystem( ) + ": setLedMode " + mode);
    m_table.getEntry("ledMode").setValue(mode);
  }

  public int getLEDMode( )
  {
    int mode = m_table.getEntry("ledMode").getNumber(0.0).intValue( );

    DataLogManager.log(getSubsystem( ) + "getLedMode :" + mode);
    return mode;
  }

  public void setCameraDisplay(int stream)
  {
    DataLogManager.log(getSubsystem( ) + ": setCameraDisplay " + stream);
    m_table.getEntry("stream").setValue(stream);
  }

  private double calculateDist(double vertAngle)
  {
    return (m_slope * vertAngle) + m_offset;
  }

  public void syncStateFromDashboard( )
  {
    m_distance1 = SmartDashboard.getNumber("VI_distance1", m_distance1);
    m_distance2 = SmartDashboard.getNumber("VI_distance2", m_distance2);
    m_vertOffset1 = SmartDashboard.getNumber("VI_vertOffset1", m_vertOffset1);
    m_vertOffset2 = SmartDashboard.getNumber("VI_vertOffset2", m_vertOffset2);

    m_slope = (m_distance2 - m_distance1) / (m_vertOffset2 - m_vertOffset1);
    m_offset = m_distance1 - m_slope * m_vertOffset1;

    SmartDashboard.putNumber("VI_Slope", m_slope);
    SmartDashboard.putNumber("VI_Offset", m_offset);
  }

}
