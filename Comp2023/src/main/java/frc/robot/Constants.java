
package frc.robot;

import java.util.Collections;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants
{
  // Toggles constants between comp robot and practice robot (named "beta")
  public static boolean      isComp;

  // bot serial nums
  public static final String kCompSN           = "03238074";
  public static final String kBetaSN           = "03260A3A";

  // Game controller definitions
  public static final int    kDriverPadPort    = 0;
  public static final int    kOperatorPadPort  = 1;

  public static final double kStickDeadband    = 0.15;
  public static final double kTriggerThreshold = 0.25;

  // Timeout constants
  public static final int    kLongCANTimeoutMs = 100;
  public static final int    kCANTimeoutMs     = 10;

  // PDH constatns
  public static final double kMinCurrent       = 0.125;  // Minimum detectable current from PDH

  // Field constants
  public static final double kGridNodeSpacing  = 0.4667; // 18.375 inches between cube node and peg nodes

  // CAN IDs and PWM IDs

  public static final class Ports
  {
    public static final String kCANCarnivore     = "canivore1";
    public static final String kCANRio           = "rio";

    // CANivore CAN IDs - Swerve
    public static final int    kCANID_DriveLF    = 1;
    public static final int    kCANID_TurnLF     = 2;
    public static final int    kCANID_CANCoderLF = 3;

    public static final int    kCANID_DriveRF    = 4;
    public static final int    kCANID_TurnRF     = 5;
    public static final int    kCANID_CANCoderRF = 6;

    public static final int    kCANID_DriveLR    = 7;
    public static final int    kCANID_TurnLR     = 8;
    public static final int    kCANID_CANCoderLR = 9;

    public static final int    kCANID_DriveRR    = 10;
    public static final int    kCANID_TurnRR     = 11;
    public static final int    kCANID_CANCoderRR = 12;

    public static final int    kCANID_Pigeon2    = 13;

    // RoboRIO CAN IDs
    public static final int    kCANID_Elbow      = 15;
    public static final int    kCANID_ELCANCoder = 16;

    public static final int    kCANID_Extension  = 17;

    public static final int    kCANID_Wrist      = 19;
    public static final int    kCANID_WRCANCoder = 20;

    public static final int    kCANID_Gripper    = 21;

    public static final int    kCANID_CANdle     = 0;

    // Digital I/Os
    // public static final int    kDIO_ExampleDetect = 2;
  }

  public static final class Falcon500
  {
    public static int          kMaxRPM           = 6380;             // free speed for Falcon 500 motor
    public static final double kEncoderCPR       = 2048;             // CPR is 2048 from Falcon 500 Manual
    public static final int    kTalonReqVersion  = ((22 * 256) + 0); // Talon version is 23.0
    public static final int    kPigeonReqVersion = ((22 * 256) + 0); // Pigeon IMU version is 23.0

    // // Input current limit settings - default
    // public static final double kSupplyCurrentLimit   = 45.0;  // Default supply current limit (after trigger)
    // public static final double kSupplyTriggerCurrent = 45.0;  // Trigger current that will cause limiting
    // public static final double kSupplyTriggerTime    = 0.001; // Time duration of trigger that will causing limiting

    // // Output current limit settings - default
    // public static final double kStatorCurrentLimit   = 80.0;  // Default supply current limit (after trigger)
    // public static final double kStatorTriggerCurrent = 80.0;  // Default trigger current that will cause limiting
    // public static final double kStatorTriggerTime    = 0.001; // Default time duration of trigger that will causing limiting
  }

  public static final class SWConsts
  {
    // Constants for balance
    public static final double kDriveBalancedAngle  = 5.0;    // Pitch values less than this stop driving
    public static final double kDriveBalanceKp      = -0.045;  // Amount of power to apply per degree

    // Limelight PID driving controls
    public static final double kTurnConstant        = 0.0;
    public static final double kTurnPidKp           = 0.005;
    public static final double kTurnPidKi           = 0.0;
    public static final double kTurnPidKd           = 0.0;
    public static final double kTurnMax             = 0.4;
    public static final double kThrottlePidKp       = 0.011;
    public static final double kThrottlePidKi       = 0.0;
    public static final double kThrottlePidKd       = 0.0;
    public static final double kThrottleMax         = 0.2;
    public static final double kThrottleShape       = 10.0;

    public static final double kTargetAngle         = 0.0;      // Optimal shooting angle
    public static final double kSetPointDistance    = 60.0;     // Optimal shooting distance
    public static final double kAngleThreshold      = 3.5;      // Degrees tolerance around optimal
    public static final double kDistThreshold       = 6.0;      // Inches tolerance around optimal

    public static final double kElbowDriveSlowAngle = 20.0;     // When arm is out beyond this angle - drive is slowed down
  }

  public static final class ELConsts
  {
    // Global settings

    public static final double  kElbowGearRatio       = 300;    // Gear reduction for elbow
    public static final double  kElbowDegreesPerCount = 360 / Falcon500.kEncoderCPR / kElbowGearRatio;
    public static final double  kForearmLengthMeters  = 1.22;   // Sim value: 48 inches
    public static final double  kForearmMassKg        = 6.0;    // Sim value: 13.2 lbs 
    public static final boolean kInvertMotor          = true;   // Motor direction for positive input

    public static final double  kElbowAngleMin        = -3.0;   // Elbow minimum allowable degrees
    public static final double  kElbowAngleMax        = 115.0;  // Elbow maximum allowable degrees
    public static final double  kElbowAngleStow       = 2.0;    // TODO: FIGURE IT OUT EMPIRICALLY
    public static final double  kElbowAngleIdle       = 20.0;   // TODO: FIGURE IT OUT EMPIRICALLY
    public static final double  kElbowAngleScoreLow   = 30.0;   // From Mech Design (floor, feet art 5" high)
    public static final double  kElbowAngleScoreMid   = 91.0;   // From Mech Design (1'10-3/4" deep, 2'10" high peg, 1'11-1/2 high cube)
    public static final double  kElbowAngleScoreHigh  = 103.0;  // From Mech Design (3'3-3/4" deep, 3'10" high peg, 2'11-1/2 high cube)
    public static final double  kElbowAngleSubstation = 103.0;  // From Mech Design (3'1-38" above floor)

    // Current limit settings - elbow
    public static final double  kSupplyCurrentLimit   = 25.0;  // Supply current limit (after trigger)
    public static final double  kSupplyTriggerCurrent = 25.0;  // Supply trigger current that will cause limiting
    public static final double  kSupplyTriggerTime    = 0.001; // Supply time duration of trigger that will causing limiting
    public static final double  kStatorCurrentLimit   = 25.0;  // Stator current limit (after trigger)
    public static final double  kStatorTriggerCurrent = 25.0;  // Stator trigger current that will cause limiting
    public static final double  kStatorTriggerTime    = 0.001; // Stator time duration of trigger that will causing limiting

    // CANCoder elbow absolute offset
    public static final boolean kInvertCANCoder       = true;   // CANCoder direction for positive angle in relative mode
    public static final boolean kElbowCalibrated      = true;   // Indicates whether the elbow has been calibrated by CANCoder
    public static final double  kCompElbowOffset      = 132.363; // CANCoder offset angle for comp bot
    public static final double  kBetaElbowOffset      = 0.000; // (TODO: Beta requires an offset) CANCoder offset angle for beta bot

    // Manual config parameters

    public enum ElbowMode
    {
      ELBOW_INIT,         // Initialize elbow
      ELBOW_DOWN,         // Elbow moving down
      ELBOW_STOPPED,      // Elbow stop and hold position
      ELBOW_UP            // Elbow moving up
    }

    public static final double kElbowSpeedMaxManual = 0.3;    // Motor percent output during manual operation

    // Motion Magic config parameters

    public enum ElbowAngle
    {
      ELBOW_NOCHANGE,     // No change in elbow angle--maintain current position
      ELBOW_STOW,         // Move elbow to stow angle
      ELBOW_IDLE,         // Move elbow to idle andgle
      ELBOW_LOW,          // Move elbow to low-scoring angle
      ELBOW_MID,          // Move elbow to shelf Angle; slightly higher than mid-scoring angle so this is used for both
      ELBOW_HIGH,         // Move elbow to high-scoring angle
      ELBOW_SHELF         // Move elbow to substation loading shelf angle
    }

    public static final int    kElbowMMVelocity       = 16646;  // Elbow motion magic velocity
    public static final int    kElbowMMAcceleration   = 16646;  // Elbow motion magic acceleration
    public static final int    kElbowMMSCurveStrength = 0;      // Elbow motion magic S curve smoothing strength
    public static final double kElbowPidKf            = 0.0461; // Elbow PID force constant
    public static final double kElbowPidKp            = 0.0246; // Elbow PID proportional constant
    public static final double kElbowPidKi            = 0.0;    // Elbow PID integral constant
    public static final double kElbowPidKd            = 0.0;    // Elbow PID derivative constant
    public static final int    kElbowAllowedError     = 0;      // Elbow PID allowable closed loop error in counts
    public static final double kElbowToleranceDegrees = 1.0;    // Elbow PID tolerance in degrees (1 deg is 1" at 48" extension)
    public static final double kElbowArbitraryFF      = 0.070;  // Elbow motor output for arm at 90 degrees
  }

  public static final class EXConsts
  {
    // Global settings

    public static final double  kExtensionGearRatio        = 18.23;   // Gear reduction for extension
    public static final double  kDrumDiameterInches        = 1.375;  // Drum diameter in inches
    public static final double  kDrumCircumInches          = kDrumDiameterInches * Math.PI;            // Drum diameter in inches
    public static final double  kRolloutRatio              = kDrumCircumInches / kExtensionGearRatio;  // inches per shaft rotation
    public static final double  kExtensionInchesPerCount   = kRolloutRatio / Falcon500.kEncoderCPR;
    // public static final double  kForearmLengthMeters        = 1.22;   // Sim value: 48 inches (no sim for extension)
    // public static final double  kForearmMassKg              = 6.0;    // Sim value: 13.2 lbs 
    public static final boolean kInvertMotor               = true;  // Motor direction for positive input
    public static final double  kSpeedCalibrate            = -0.12;   // Motor percent output during calibration

    public static final double  kExtensionLengthMin        = -0.5;   // Extension minimum allowable length
    public static final double  kExtensionLengthMax        = 20.0;   // Extension maximum allowable length
    public static final double  kExtensionLengthStow       = 0.0;    // TODO: FIGURE IT OUT EMPIRICALLY
    public static final double  kExtensionLengthIdle       = 0.0;    // TODO: FIGURE IT OUT EMPIRICALLY
    public static final double  kExtensionLengthScoreLow   = 8.0;    // From Mech Design (floor, feet art 5" high)
    public static final double  kExtensionLengthScoreMid   = 3.0;    // From Mech Design (1'10-3/4" deep, 2'10" high peg, 1'11-1/2 high cube)
    public static final double  kExtensionLengthScoreHigh  = 18.0;   // From Mech Design (3'3-3/4" deep, 3'10" high peg, 2'11-1/2 high cube)
    public static final double  kExtensionLengthSubstation = 0.0;    // From Mech Design (3'1-38" above floor)

    // Current limit settings - extension
    public static final double  kSupplyCurrentLimit        = 15.0;  // Supply current limit (after trigger)
    public static final double  kSupplyTriggerCurrent      = 15.0;  // Supply trigger current that will cause limiting
    public static final double  kSupplyTriggerTime         = 0.001; // Supply time duration of trigger that will causing limiting
    public static final double  kStatorCurrentLimit        = 15.0;  // Stator current limit (after trigger)
    public static final double  kStatorTriggerCurrent      = 15.0;  // Stator trigger current that will cause limiting
    public static final double  kStatorTriggerTime         = 0.001; // Stator time duration of trigger that will causing limiting

    // CANCoder extension absolute offset
    public static final boolean kInvertCANCoder            = false;  // CANCoder direction for positive angle in relative mode
    public static final boolean kExtensionCalibrated       = false;  // Indicates whether the extension has been calibrated by CANCoder
    public static final double  kCompExtensionOffset       = 0.000;  // CANCoder offset for comp bot
    public static final double  kBetaExtensionOffset       = 0.000;  // CANCoder offset for beta bot

    // Manual config parameters

    public enum ExtensionMode
    {
      EXTENSION_INIT,         // Initialize extension
      EXTENSION_OUT,         // Extension moving out
      EXTENSION_STOPPED,      // Extension stop and hold position
      EXTENSION_IN            // Extension moving in
    }

    public static final double kExtensionSpeedMaxManual = 0.3;    // Motor percent output during manual operation

    // Motion Magic config parameters

    public enum ExtensionLength
    {
      EXTENSION_NOCHANGE,     // No change in extension length--maintain current position
      EXTENSION_STOW,         // Move extension to stow length
      EXTENSION_IDLE,         // Move extension to idle length
      EXTENSION_LOW,          // Move extension to low-scoring length
      EXTENSION_MID,          // Move extension to shelf length
      EXTENSION_HIGH,         // Move extension to high-scoring length
      EXTENSION_SHELF,        // Move extension to high-scoring length
    }

    public static final int    kExtensionMMVelocity       = 16646 / 2;  // Extension motion magic velocity
    public static final int    kExtensionMMAcceleration   = 16646 / 2;  // Extension motion magic acceleration
    public static final int    kExtensionMMSCurveStrength = 0;      // Extension motion magic S curve smoothing strength
    public static final double kExtensionPidKf            = 0.0461; // Extension PID force constant
    public static final double kExtensionPidKp            = 0.0246; // Extension PID proportional constant
    public static final double kExtensionPidKi            = 0.0;    // Extension PID integral constant
    public static final double kExtensionPidKd            = 0.0;    // Extension PID derivative constant
    public static final int    kExtensionAllowedError     = 0;      // Extension PID allowable closed loop error in counts
    public static final double kExtensionToleranceInches  = 0.5;    // Extension PID tolerance in inches
    public static final double kExtensionArbitraryFF      = -0.032; // Extension motor output for extension when fully retracted
  }

  public static final class WRConsts
  {
    // Global settings

    public static final double  kWristGearRatio       = 375;   // Gear reduction for wrist
    public static final double  kWristDegreesPerCount = 360 / Falcon500.kEncoderCPR / kWristGearRatio;
    public static final double  kGripperLengthMeters  = 0.3;   // Sim value: 11.8 in
    public static final double  kGripperMassKg        = 3.0;   // Sim value: 6.6 lbs
    public static final boolean kInvertMotor          = false; // Motor direction for positive input

    public static final double  kWristMinAngle        = -2.0;  // Wrist minimum allowable Angle
    public static final double  kWristMaxAngle        = 115.0; // Wrist maximum allowable Angle
    public static final double  kWristAngleStow       = 0.0;   // TO-DO: FIGURE IT OUT
    public static final double  kWristAngleIdle       = 5.0;   // TO-DO: FIGURE IT OUT
    public static final double  kWristAngleScoreLow   = 30.0;  // From Mech Design (floor, feet art 5" high)
    public static final double  kWristAngleScoreMid   = 91.0;  // From Mech Design (1'10-3/4" deep, 2'10" high peg, 1'11-1/2 high cube)
    public static final double  kWristAngleScoreHigh  = 103.0; // From Mech Design (3'3-3/4" deep, 3'10" high peg, 2'11-1/2 high cube)
    public static final double  kWristAngleSubstation = 103.0; // From Mech Design (3'1-38" above floor)

    // Current limit settings - wrist
    public static final double  kSupplyCurrentLimit   = 15.0;  // Supply current limit (after trigger)
    public static final double  kSupplyTriggerCurrent = 15.0;  // Supply trigger current that will cause limiting
    public static final double  kSupplyTriggerTime    = 0.001; // Supply time duration of trigger that will causing limiting
    public static final double  kStatorCurrentLimit   = 10.0;  // Stator current limit (after trigger)
    public static final double  kStatorTriggerCurrent = 10.0;  // Stator trigger current that will cause limiting
    public static final double  kStatorTriggerTime    = 0.001; // Stator time duration of trigger that will causing limiting

    // CANCoder wrist absolute offset
    public static final boolean kInvertCANCoder       = true;  // CANCoder direction for positive angle in relative mode
    public static final boolean kWristCalibrated      = true;  // Indicates whether the wrist has been calibrated by CANCoder
    public static final double  kCompWristOffset      = -48.4;  // CANCoder offset angle for comp bot
    public static final double  kBetaWristOffset      = 0.000;  // CANCoder offset angle for beta bot

    // Manual config parameters

    public enum WristMode
    {
      WRIST_INIT,         // Initialize wrist
      WRIST_DOWN,         // Wrist moving down
      WRIST_STOPPED,      // Wrist stop and hold position
      WRIST_UP            // Wrist moving up
    }

    public static final double kWristSpeedMaxManual = 0.3;    // Motor percent output during manual operation

    // Motion Magic config parameters

    public enum WristAngle
    {
      WRIST_NOCHANGE,     // No change in Wrist Angle--maintain current position
      WRIST_STOW,         // Move wrist to stow position
      WRIST_IDLE,         // Move wrist to stow position
      WRIST_LOW,          // Move wrist to low-scoring Angle
      WRIST_MID,          // Move wrist to shelf Angle; slightly higher than mid-scoring Angle so this is used for both
      WRIST_HIGH,         // Move wrist to high-scoring Angle
      WRIST_SHELF         // Move wrist to substation loading shelf angle
    }

    public static final int    kWristMMVelocity       = 16466;  // Wrist motion magic velocity
    public static final int    kWristMMAcceleration   = 16466;  // Wrist motion magic acceleration
    public static final int    kWristMMSCurveStrength = 0;      // Wrist motion magic S curve smoothing strength
    public static final double kWristPidKf            = 0.0466; // Wrist PID force constant
    public static final double kWristPidKp            = 0.069;  // Wrist PID proportional constant
    public static final double kWristPidKi            = 0.0;    // Wrist PID integral constant
    public static final double kWristPidKd            = 0.0;    // Wrist PID derivative constant
    public static final int    kWristAllowedError     = 0;      // Wrist PID allowable closed loop error in counts
    public static final double kWristToleranceDegrees = 1.0;    // Wrist PID tolerance in degrees (1 deg is 0.25" at 15" length)
    public static final double kWristArbitraryFF      = 0.058;  // Wrist motor output for 90 degrees
  }

  public static final class GRConsts
  {
    // Global settings

    public static final boolean kInvertMotor          = false;  // Motor direction for positive input

    // Input current limit settings - gripper
    public static final double  kSupplyCurrentLimit   = 30.0;  // Default supply current limit (after trigger)
    public static final double  kSupplyTriggerCurrent = 30.0;  // Trigger current that will cause limiting
    public static final double  kSupplyTriggerTime    = 0.001; // Time duration of trigger that will causing limiting

    public enum GRMode
    {
      GR_STOP,    // stop motor
      GR_ACQUIRE, // acquire game pieces
      GR_EXPEL,   // expel game pieces
      GR_HOLD,    // hold game pieces
    }

    public static final double kGripperSpeedAcquire = 1.0;  // Acquire game piece from loading station or floor
    public static final double kGripperSpeedHold    = 0.1;  // Hold game piece while traversing the field (must be < 2V equiv)
    public static final double kGripperSpeedExpel   = -0.3; // Score game piece on cone node or cube shelf
  }

  public static final class LEDConsts
  {
    public enum LEDColor
    {
      LEDCOLOR_OFF,     // CANdle off
      LEDCOLOR_WHITE,   // CANdle white
      LEDCOLOR_RED,     // CANdle red
      LEDCOLOR_ORANGE,  // CANdle orange
      LEDCOLOR_YELLOW,  // CANdle yellow
      LEDCOLOR_GREEN,   // CANdle green
      LEDCOLOR_BLUE,    // CANdle blue
      LEDCOLOR_PURPLE,  // CANdle purple
      LEDCOLOR_DASH     // CANdle color taken from dashboard
    }
  }

  public static final class VIConsts
  {
    // Limelight-defined streaming states
    public static final int STANDARD      = 0;  // Both cameras side-by-side
    public static final int PIP_MAIN      = 1;  // Limelight with second camera inset
    public static final int PIP_SECONDARY = 2;  // Second camera with limelight inset

    // Limelight-defined LED mode states
    public static final int LED_OFF       = 1;
    public static final int LED_ON        = 3;

    public enum VIRequests
    {
      VISION_OFF,   // Disable limelight LED and enable secondary camera mode
      VISION_ON,    // Enable limelight LED and disable secondary camera mode
      VISION_TOGGLE // Toggle modes
    }

    // Direction of goal relative to AprilTag 
    public enum VIGoalDirection
    {
      DIRECTION_LEFT,   // Left
      DIRECTION_MIDDLE, // Middle
      DIRECTION_RIGHT   // Right
    }

    public static final double       kLLDistance1        = 48;    // distance from bumper in inches for first reference point
    public static final double       kLLVertOffset1      = 0.42;  // LL y reading in degrees for first reference point
    public static final double       kLLDistance2        = 60;    // distance from bumper in inches for second reference point
    public static final double       kLLVertOffset2      = -4.85; // LL y reading in degrees for second reference point

    public static final double       kATagDepthInGrid    = Units.inchesToMeters(14.25); // Depth from front of grid to AprilTag - 1'2-1/4"
    public static final double       kRobotCenterToFront = Units.inchesToMeters((28.0 + 6.0) / 2); // Depth from limelight to front robot edge
    public static final double       kAdjustPathX        = kATagDepthInGrid + kRobotCenterToFront;
    public static final double       kAdjustPathY        = Units.inchesToMeters(18.25 / 2 + 18.5 / 2) + 0.06; // Addition of 6cm to adjust for empirical error 
    public static final double       kAdjustSubPathX     = kRobotCenterToFront + Units.inchesToMeters(30); //Robot stop 30 inches from the substation loading zone
    public static final double       kAdjustSubPathY     = Units.inchesToMeters(50.5 / 2);

    public static final List<Pose2d> kAprilTagPoses      = Collections.unmodifiableList(List.of( //
        new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0)),        // AprilTag ID: 0 (invalid)
        new Pose2d(new Translation2d(15.513558, 1.071626), new Rotation2d(Units.degreesToRadians(180))), // AprilTag ID: 1 
        new Pose2d(new Translation2d(15.513558, 2.748026), new Rotation2d(Units.degreesToRadians(180))), // AprilTag ID: 2 
        new Pose2d(new Translation2d(15.513558, 4.424426), new Rotation2d(Units.degreesToRadians(180))), // AprilTag ID: 3 
        new Pose2d(new Translation2d(16.178784, 6.749796), new Rotation2d(Units.degreesToRadians(180))), // AprilTag ID: 4 
        new Pose2d(new Translation2d(0.36195, 6.749796), new Rotation2d(0)),   // AprilTag ID: 5 
        new Pose2d(new Translation2d(1.0273, 4.424426), new Rotation2d(0)),    // AprilTag ID: 6 
        new Pose2d(new Translation2d(1.0273, 2.748026), new Rotation2d(0)),    // AprilTag ID: 7
        new Pose2d(new Translation2d(1.0273, 1.071626), new Rotation2d(0))     // AprilTag ID: 8
    ));
  }

  //// 1678 Constants ///////////////////////////////////////////////////////////

  public static final class SwerveConstants
  {
    public static final boolean                                      invertGyro                  = false; // Always ensure Gyro is CCW+ CW-

    /* Swerve Constants - 0.427 m (x, y) */
    public static final double                                       trackWidth                  = Units.inchesToMeters(22.7);
    public static final double                                       wheelBase                   = Units.inchesToMeters(22.7);

    public static final double                                       wheelDiameter               = Units.inchesToMeters(4.0);
    public static final double                                       wheelCircumference          = wheelDiameter * Math.PI;

    public static final double                                       openLoopRamp                = 0.25;
    public static final double                                       closedLoopRamp              = 0.0;

    public static final double                                       driveGearRatio              = 6.75;
    public static final double                                       angleGearRatio              = 21.43;

    public static final edu.wpi.first.math.geometry.Translation2d[ ] swerveModuleLocations       =
    {
        new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    };

    public static final SwerveDriveKinematics                        swerveKinematics            =
        new SwerveDriveKinematics(swerveModuleLocations);

    /* Swerve Current Limiting */
    public static final int                                          angleContinuousCurrentLimit = 25;
    public static final int                                          anglePeakCurrentLimit       = 40;
    public static final double                                       anglePeakCurrentDuration    = 0.1;
    public static final boolean                                      angleEnableCurrentLimit     = true;

    public static final int                                          driveContinuousCurrentLimit = 35;
    public static final int                                          drivePeakCurrentLimit       = 60;
    public static final double                                       drivePeakCurrentDuration    = 0.1;
    public static final boolean                                      driveEnableCurrentLimit     = true;

    /* Angle Motor PID Values */
    public static final double                                       angleKP                     = 0.3;
    public static final double                                       angleKI                     = 0.0;
    public static final double                                       angleKD                     = 0.0;
    public static final double                                       angleKF                     = 0.0;

    /* Drive Motor PID Values */
    public static final double                                       driveKP                     = 0.05;
    public static final double                                       driveKI                     = 0.0;
    public static final double                                       driveKD                     = 0.0;
    public static final double                                       driveKF                     = 0.0;

    /* Drive Motor Characterization Values */
    public static final double                                       driveKS                     = (0.32 / 12);
    public static final double                                       driveKV                     = (1.51 / 12);
    public static final double                                       driveKA                     = (0.27 / 12);

    /* Swerve Profiling Values */
    public static final double                                       maxSpeed                    = 4.5; // meters per second
    public static final double                                       maxAngularVelocity          = 6.0; //orginially 10.0
    public static final double                                       maxSpeedSlowMode            = 2.25; // meters per second
    public static final double                                       maxAngularVelocitySlowMode  = 4.0; //orginially 5.0

    /* Neutral Modes */
    public static final NeutralMode                                  angleNeutralMode            = NeutralMode.Coast;
    public static final NeutralMode                                  driveNeutralMode            = NeutralMode.Brake;

    /* Motor Inverts */
    public static final boolean                                      driveMotorInvert            = true;
    public static final boolean                                      angleMotorInvert            = true;

    /* Angle Encoder Invert */
    public static final boolean                                      canCoderInvert              = false;

    /* Controller Invert */
    public static final boolean                                      invertXAxis                 = false;
    public static final boolean                                      invertYAxis                 = false;
    public static final boolean                                      invertRAxis                 = false;

    /*** MODULE SPECIFIC CONSTANTS ***/

    /* Front Left Module - Module 0 */
    public static final class Mod0
    {
      public static final double epsilonAngleOffset = 325.723;
      public static final double compAngleOffset    = 15.732;

      public static SwerveModuleConstants SwerveModuleConstants( )
      {
        return new SwerveModuleConstants(Ports.kCANID_DriveLF, Ports.kCANID_TurnLF, Ports.kCANID_CANCoderLF,
            isComp ? compAngleOffset : epsilonAngleOffset);
      }
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1
    {
      public static final double epsilonAngleOffset = 142.91;
      public static final double compAngleOffset    = 239.766;

      public static SwerveModuleConstants SwerveModuleConstants( )
      {
        return new SwerveModuleConstants(Ports.kCANID_DriveRF, Ports.kCANID_TurnRF, Ports.kCANID_CANCoderRF,
            isComp ? compAngleOffset : epsilonAngleOffset);
      }
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2
    {
      public static final double epsilonAngleOffset = 137.988;
      public static final double compAngleOffset    = 98.174;

      public static SwerveModuleConstants SwerveModuleConstants( )
      {
        return new SwerveModuleConstants(Ports.kCANID_DriveLR, Ports.kCANID_TurnLR, Ports.kCANID_CANCoderLR,
            isComp ? compAngleOffset : epsilonAngleOffset);
      }
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3
    {
      public static final double epsilonAngleOffset = 44.736;
      public static final double compAngleOffset    = 93.076;

      public static SwerveModuleConstants SwerveModuleConstants( )
      {
        return new SwerveModuleConstants(Ports.kCANID_DriveRR, Ports.kCANID_TurnRR, Ports.kCANID_CANCoderRR,
            isComp ? compAngleOffset : epsilonAngleOffset);
      }
    }
  }

  public static final class SnapConstants
  {
    public static final double                       kP                                      = 5.0;
    public static final double                       kI                                      = 0;
    public static final double                       kD                                      = 0.0;
    public static final double                       kTimeout                                = 0.75;
    public static final double                       kEpsilon                                = 1.0;

    // Constraints for the profiled angle controller
    public static final double                       kMaxAngularSpeedRadiansPerSecond        = 2.0 * Math.PI;
    public static final double                       kMaxAngularSpeedRadiansPerSecondSquared =
        Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints             =
        new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class AutoConstants
  {
    public static final double                       kSlowSpeedMetersPerSecond                   = 1.7;
    public static final double                       kSlowAccelerationMetersPerSecondSquared     = 2.0;

    public static final double                       kMaxSpeedMetersPerSecond                    = 2.2;
    public static final double                       kMaxAccelerationMetersPerSecondSquared      = 2.3;

    public static final double                       kSlowMaxAngularSpeedRadiansPerSecond        = 0.8 * Math.PI;
    public static final double                       kSlowMaxAngularSpeedRadiansPerSecondSquared =
        Math.pow(kSlowMaxAngularSpeedRadiansPerSecond, 2);

    public static final double                       kMaxAngularSpeedRadiansPerSecond            = 1.2 * Math.PI;
    public static final double                       kMaxAngularSpeedRadiansPerSecondSquared     =
        Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

    public static final double                       kPXController                               = 1;
    public static final double                       kPYController                               = 1;
    public static final double                       kPThetaController                           = 5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints                 =
        new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kSlowThetaControllerConstraints             =
        new TrapezoidProfile.Constraints(kSlowMaxAngularSpeedRadiansPerSecond, kSlowMaxAngularSpeedRadiansPerSecondSquared);

    public static TrajectoryConfig createConfig(double maxSpeed, double maxAccel, double startSpeed, double endSpeed)
    {
      TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAccel);
      config.setKinematics(Constants.SwerveConstants.swerveKinematics);
      config.setStartVelocity(startSpeed);
      config.setEndVelocity(endSpeed);
      config.addConstraint(new CentripetalAccelerationConstraint(3.0));
      return config;
    }

    // Trajectory Speed Configs
    public static final TrajectoryConfig defaultSpeedConfig =
        new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.SwerveConstants.swerveKinematics);

    public static final TrajectoryConfig slowSpeedConfig    =
        new TrajectoryConfig(kSlowSpeedMetersPerSecond, kSlowAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.SwerveConstants.swerveKinematics).setStartVelocity(0).setEndVelocity(0);
  }

  //// 1678 Constants ///////////////////////////////////////////////////////////
}
