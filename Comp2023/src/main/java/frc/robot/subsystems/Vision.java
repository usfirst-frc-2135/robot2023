// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.VIConsts;

/**
 *
 */
public class Vision extends SubsystemBase
{
  // Objects
  public MedianFilter           m_tyfilter           = new MedianFilter(5); // median filter y values to remove outliers (5 sample)
  public MedianFilter           m_tvfilter           = new MedianFilter(5); // median filter v values to remove outliers (5 sample)

  // Declare module variables
  private double                m_distance1          = VIConsts.kLLDistance1;   // x position in inches for first reference point
  private double                m_vertOffset1        = VIConsts.kLLVertOffset1; // y reading in degrees for first reference point
  private double                m_distance2          = VIConsts.kLLDistance2;   // x position in inches for second reference point
  private double                m_vertOffset2        = VIConsts.kLLVertOffset2; // y reading in degrees for second reference point
  private double                m_slope;   // Linear regressions slope from calibration
  private double                m_offset;  // Linear regressions slope from calibration

  private NetworkTable          m_table;            // Network table reference for getting LL values

  private DoubleArraySubscriber m_botposeSub;
  private Transform3d           m_botposeTransform3d = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
  private double[ ]             m_sendablePoseArray  = new double[3];

  private double                m_yawBotPose         = 0; // yaw of Robot based on apriltag calculated position

  private double                m_targetHorizAngle; // LL Target horizontal Offset from Crosshair to Target (-27 to 27 deg)
  private double                m_targetVertAngle;  // LL Target vertical Offset from Crosshair to Target (-20.5 to 20.5 deg)
  private double                m_targetArea;       // LL Target Area (0% of image to 100% of image)
  private double                m_targetSkew;       // LL Target Skew or rotation (-90 to 0 deg)
  private boolean               m_targetValid;      // LL Target Valid or not
  private double                m_targetLatency;    // LL pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
  private double                m_targetID;         // ID of the primary in-view AprilTag

  private double                m_distLL;           // calculated distance in inches for the current y value

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
    setLEDMode(VIConsts.LED_OFF);

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

    SmartDashboard.putNumberArray("VI_RobotPose", new double[ ] { });
    SmartDashboard.putNumber("VI_area", m_targetArea);
    SmartDashboard.putNumber("VI_skew", m_targetSkew);
    SmartDashboard.putBoolean("VI_valid", m_targetValid);
    SmartDashboard.putNumber("VI_targetLatency", m_targetLatency);

    m_botposeSub = m_table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[ ] { });

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
      m_targetID = m_table.getEntry("tid").getDouble(-1.0);
      m_targetLatency = m_table.getEntry("tl").getDouble(0.0);
    }

    m_distLL = calculateDist(m_targetVertAngle);

    if (m_targetID > 0)
    {
      double[ ] m_botposeArray = m_botposeSub.get( );

      if (m_botposeArray != null)
      {
        //Defining the Transform3d of the robot
        m_botposeTransform3d = new Transform3d(new Translation3d(m_botposeArray[0], m_botposeArray[1], m_botposeArray[2]),
            new Rotation3d(m_botposeArray[3], m_botposeArray[4], m_botposeArray[5]));

        m_sendablePoseArray[0] = m_botposeArray[0]; // X pose from the Translation3d of the "botPose"
        m_sendablePoseArray[1] = m_botposeArray[1]; // Y pose from the Translation3d of the "botPose"
        m_sendablePoseArray[2] = m_botposeArray[5]; // Rotation from yaw from the Rotation3d of the "botPose"

        m_yawBotPose = m_botposeArray[5]; //Setting the rotation of the robot
      }

    }

    SmartDashboard.putData("Field", RobotContainer.getInstance( ).m_field2d);
    SmartDashboard.putNumberArray("VI_RobotPose", m_sendablePoseArray);

    SmartDashboard.putNumber("VI_horizAngle", m_targetHorizAngle);
    SmartDashboard.putNumber("VI_vertAngle", m_targetVertAngle);
    SmartDashboard.putNumber("VI_area", m_targetArea);
    SmartDashboard.putNumber("VI_skew", m_targetSkew);
    SmartDashboard.putBoolean("VI_valid", m_targetValid);
    SmartDashboard.putNumber("VI_targetLatency", m_targetLatency);

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

    //setLEDMode(VIConsts.LED_OFF);
    //setCameraDisplay(VIConsts.PIP_SECONDARY);

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

  public double getTargetID( )
  {
    return m_targetID;
  }

  public double getTargetLatency( )
  {
    return m_targetLatency;
  }

  public Transform3d getBotPoseTransform3d( )
  {
    return m_botposeTransform3d;
  }

  public Pose2d getBotPose2d(boolean checkPosition)
  {
    Translation2d tran;
    Rotation2d rot;

    // Perform valid checks and return null if it doesn't exist
    if (m_targetValid)
    {
      tran = new Translation2d(m_botposeTransform3d.getX( ), m_botposeTransform3d.getY( ));
      double degrees = m_yawBotPose + ((m_yawBotPose < 0) ? 360 : 0);
      rot = new Rotation2d(Units.degreesToRadians(degrees));
      if (checkPosition)
        if (isBotPoseValid(tran))
        {
          return new Pose2d(tran, rot);
        }
      if (!checkPosition)
      {
        return new Pose2d(tran, rot);
      }

    }

    // DataLogManager.log(getSubsystem( )                                         //  
    //     + "VISION: Radians sent " + rot                                        //
    //     + " from the limelight " + m_botposeTransform3d.getRotation( ).getZ( ) //
    //     + " Angle : " + m_botposeTransform3d.getRotation( ).getAngle( )        //
    // );

    return null;
  }

  public boolean isBotPoseValid(Translation2d botpose)
  {
    //Only adding vision measurements that are already within one meter or so of the current pose estimate
    Translation2d currentPose = RobotContainer.getInstance( ).m_swerve.m_poseEstimator.getEstimatedPosition( ).getTranslation( );

    if (Math.abs(currentPose.getX( ) - botpose.getX( )) <= 1)
    {
      if (Math.abs(currentPose.getY( ) - botpose.getY( )) <= 1)
      {
        return true;
      }
    }

    return false;
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
