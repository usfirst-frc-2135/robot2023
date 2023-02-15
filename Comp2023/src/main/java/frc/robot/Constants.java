
package frc.robot;

import java.util.Collections;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
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
  // Game controller definitions
  public static final int    kDriverPadPort    = 0;
  public static final int    kOperatorPadPort  = 1;

  public static final double kStickDeadband    = 0.15;
  public static final double kTriggerThreshold = 0.25;

  // CAN IDs and PWM IDs
  public static final class Ports
  {
    public static final String kCANCarnivore     = "canivore1";
    public static final String kCANRio           = "rio";

    // Swerve IDs
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

    // Other subsystem CAN IDs
    public static final int    kCANID_Elbow      = 15;
    public static final int    kCANID_ELCANCoder = 16;
    public static final int    kCANID_Wrist      = 17;
    public static final int    kCANID_WRCANCoder = 18;
    public static final int    kCANID_Gripper    = 19;

    public static final int    kCANID_CANdle     = 0;

    // Digital I/Os
    // public static final int    kDIO_CargoDetect  = 2;
  }

  public static final class Falcon500
  {
    public static int          kMaxRPM               = 6380;             // free speed for Falcon 500 motor
    public static final double kEncoderCPR           = 2048;             // CPR is 2048 from Falcon 500 Manual
    public static final int    kTalonReqVersion      = ((22 * 256) + 0); // Talon version is 23.0
    public static final int    kPigeonReqVersion     = ((22 * 256) + 0); // Pigeon IMU version is 23.0

    // Input current limit settings
    public static final double kSupplyCurrentLimit   = 45.0;  // Default supply current limit (after trigger)
    public static final double kSupplyTriggerCurrent = 45.0;  // Trigger current that will cause limiting
    public static final double kSupplyTriggerTime    = 0.001; // Time duration of trigger that will causing limiting

    // Output current limit settings
    public static final double kStatorCurrentLimit   = 80.0;  // Default supply current limit (after trigger)
    public static final double kStatorTriggerCurrent = 80.0;  // Default trigger current that will cause limiting
    public static final double kStatorTriggerTime    = 0.001; // Default time duration of trigger that will causing limiting
  }

  public static final class ARMConsts
  {
    public static final int    kARMLeftLimitDIO      = 0;
    public static final int    kARMRightLimitDIO     = 1;
    public static final int    kGateHookSolenod      = 1;
    public static final int    kARMCancoderID        = 0;

    public static final double kElbowGearRatio       = 300;   // Gear reduction for elbow
    public static final double kElbowDegreesPerCount = 360 / Falcon500.kEncoderCPR / kElbowGearRatio;
    public static final double kForearmLengthMeters  = 1.0;
    public static final double kForearmMassKg        = 6.5;
    public static final double kWristGearRatio       = 300;   // Gear reduction for wrist
    public static final double kGripperLengthMeters  = 0.3;
    public static final double kGripperMassKg        = 4.0;
    public static final double kWristDegreesPerCount = 360 / Falcon500.kEncoderCPR / kWristGearRatio;

    // Config file parameters
    public static final int    kMMVelocity           = 21776;  // Arm motion magic velocity
    public static final int    kMMAcceleration       = 43552;  // Arm motion magic acceleration
    public static final int    kMMSCurveStrength     = 0;      // Arm motion magic S curve smoothing strength
    public static final double kARMPidKf             = 0.0496; // Arm PID force constant
    public static final double kARMPidKp             = 0.500;  // Arm PID proportional constant
    public static final double kARMPidKi             = 0.0;    // Arm PID integral constant
    public static final double kARMPidKd             = 0.0;    // Arm PID derivative constant
    public static final int    kELAllowedError       = 0;      // Arm PID allowable closed loop error in counts
    public static final int    kWRAllowedError       = 0;      // Arm PID allowable closed loop error in counts
    public static final double kARMToleranceInches   = 0.25;   // Arm PID tolerance in inches

    public static final double kStowHeight           = 0.10;   // 0.25 inches
    public static final double kExtendL2             = 29.0;   // 29 inches
    public static final double kRotateL3             = 31.25;  // 21 inches
    public static final double kRaiseL4              = 15.0;   // 25.25 inches
    public static final double kGatehookRestHeight   = 4.0;    // 0.35 inches
    public static final double kArmMinHeight         = 0.0;    // Arm minimum allowable height
    public static final double kArmMaxHeight         = 36.0;   // Arm maximum allowable height

    public static final double kSpeedCalibrate       = -0.1;   // Motor percent output during calibration
    public static final double kSpeedMaxManual       = 0.3;    // Motor percent output during manual operation
    public static final double kStickDeadband        = 0.2;    // Joystick deadband for manual operaton

    public static final double kClimbL2Time          = 0.5;
    public static final double kRotateExtendL3Time   = 1.5;
    public static final double kRotateRetractL3Time  = 2.0;
    public static final double kClimbL3Time          = 0.5;
    public static final double kRotateRetractL4Time  = 2.5;

    public enum ElbowMode
    {
      ELBOW_INIT,         // Initialize elbow
      ELBOW_DOWN,         // Move elbow down
      ELBOW_STOPPED,      // Stop and hold position
      ELBOW_UP            // Move elbow up
    }

    public enum WristMode
    {
      WRIST_INIT,         // Initialize wrist
      WRIST_DOWN,         // Move wrist down
      WRIST_STOPPED,      // Stop and hold position
      WRIST_UP            // Move wrist up
    }
  }

  public static final class GRConsts
  {
    public static final double kGRAcquireSpeed = 1.0;
    public static final double kGRExpelSpeed   = -1.0;
    public static final double kGRHoldSpeed    = 0.1;

    public enum GRMode
    {
      GR_STOP,    // stop motor
      GR_ACQUIRE, // acquire game pieces
      GR_EXPEL,   // expel game pieces
      GR_HOLD,    // hold game pieces
    }
  }

  public static final class SWConsts
  {
    // Joystick tuning
    public static final double kDriveXScaling    = 1.0;           // 1.0 is no scaling
    public static final double kDriveYScaling    = 1.0;           // 1.0 is no scaling
    public static final double kDriveSlowScaling = 0.3;           // Scale by 30% of full speed

    // Teleop driving controls
    public static final double kOpenLoopRamp     = 0.5;           // CTRE: full speed in 0.5 sec
    public static final double kClosedLoopRamp   = 0.0;           // CTRE: 0 is disabled
    public static final double kStopTolerance    = 0.05;          // Target position tolerance (< 5cm)

    // Limelight driving controls
    public static final double kTurnConstant     = 0.0;
    public static final double kTurnPidKp        = 0.005;
    public static final double kTurnPidKi        = 0.0;
    public static final double kTurnPidKd        = 0.0;
    public static final double kTurnMax          = 0.4;
    public static final double kThrottlePidKp    = 0.011;
    public static final double kThrottlePidKi    = 0.0;
    public static final double kThrottlePidKd    = 0.0;
    public static final double kThrottleMax      = 0.2;
    public static final double kThrottleShape    = 10.0;

    public static final double kTargetAngle      = 0.0;           // Optimal shooting angle
    public static final double kSetPointDistance = 60.0;          // Optimal shooting distance
    public static final double kAngleThreshold   = 3.5;           // Degrees tolerance around optimal
    public static final double kDistThreshold    = 6.0;           // Inches tolerance around optimal
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

    public static final double       kLLDistance1   = 48;    // distance from bumper in inches for first reference point
    public static final double       kLLVertOffset1 = 0.42;  // LL y reading in degrees for first reference point
    public static final double       kLLDistance2   = 60;    // distance from bumper in inches for second reference point
    public static final double       kLLVertOffset2 = -4.85; // LL y reading in degrees for second reference point

    public static final List<Pose3d> kAprilTagPoses =
        Collections.unmodifiableList(List.of(new Pose3d(new Translation3d(7.24310, -2.93659, 0), new Rotation3d(0, 0, 0)), // AprilTag ID: 1 
            new Pose3d(new Translation3d(7.24310, -1.26019, 0), new Rotation3d(0, 0, 0)), // AprilTag ID: 2 
            new Pose3d(new Translation3d(7.24310, 0.41621, 0), new Rotation3d(0, 0, 0)), // AprilTag ID: 3 
            new Pose3d(new Translation3d(7.24310, 2.74161, 0), new Rotation3d(0, 0, 0)), // AprilTag ID: 4 
            new Pose3d(new Translation3d(-7.24310, 2.74161, 0), new Rotation3d(0, 0, 0)), // AprilTag ID: 5 
            new Pose3d(new Translation3d(-7.24310, 0.46272, 0), new Rotation3d(0, 0, 0)), // AprilTag ID: 6 
            new Pose3d(new Translation3d(-7.24310, -1.26019, 0), new Rotation3d(0, 0, 0)), // AprilTag ID: 7
            new Pose3d(new Translation3d(-7.24310, -2.74161, 0), new Rotation3d(0, 0, 0)) // AprilTag ID: 8
        ));
  }

  public static final class SIMLLConsts
  {
    // public static final double kFieldLength        = Units.feetToMeters(54.0);      // Field dimensions are 54ft x 27ft
    // public static final double kFieldWidth         = Units.feetToMeters(27.0);
    // public static final double kGoalPostionX       = kFieldLength / 2 - Units.feetToMeters(2.0); // Goal target on field
    // public static final double kGoalPostionY       = kFieldWidth / 2;
    // public static final double kGoalHeight         = Units.inchesToMeters(102.81);  // Upper hub height from floor
    // public static final double kCameraPositionX    = Units.inchesToMeters(0.0);     // Camera position on robot (X, Y)
    // public static final double kCameraPositionY    = Units.inchesToMeters(0.0);
    // public static final double kCameraRotation     = Units.degreesToRadians(180.0); // Camera rotation on robot
    // public static final double kCameraLensHeight   = Units.inchesToMeters(41.0);    // Camera lens height from floor
    // public static final double kCameraLensBackTilt = Units.degreesToRadians(40.0);  // Camera backward tilt from normal
  }

  //// 1678 Constants ///////////////////////////////////////////////////////////

  // toggle constants between comp bot and practice bot (named "beta")
  public static boolean   isComp;

  // Timeout constants
  public static final int kLongCANTimeoutMs = 100;
  public static final int kCANTimeoutMs     = 10;

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
    public static final double                                       maxAngularVelocity          = 10.0;

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
      public static final double compAngleOffset    = 16.172;

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
      public static final double compAngleOffset    = 239.502;

      public static SwerveModuleConstants SwerveModuleConstants( )
      {
        return new SwerveModuleConstants(Ports.kCANID_DriveRF, Ports.kCANID_TurnRF, Ports.kCANID_CANCoderRF,
            isComp ? compAngleOffset : epsilonAngleOffset);
      }
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2
    {
      public static final double epsilonAngleOffset = 227.549;
      public static final double compAngleOffset    = 97.471;

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
      public static final double compAngleOffset    = 92.549;

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
    public static final double                       kTimeout                                = 0.25;
    public static final double                       kEpsilon                                = 1.0;

    // Constraints for the profiled angle controller
    public static final double                       kMaxAngularSpeedRadiansPerSecond        = 2.0 * Math.PI;
    public static final double                       kMaxAngularSpeedRadiansPerSecondSquared =
        Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints             =
        new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class VisionAlignConstants
  {
    public static final double                       kP                                      = 6.37;
    public static final double                       kI                                      = 0.0;
    public static final double                       kD                                      = 0.10;
    public static final double                       kTimeout                                = 0.25;
    public static final double                       kEpsilon                                = 5.0;

    // Constraints for the profiled angle controller
    public static final double                       kMaxAngularSpeedRadiansPerSecond        = 2.0 * Math.PI;
    public static final double                       kMaxAngularSpeedRadiansPerSecondSquared = 10.0 * Math.PI;

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
