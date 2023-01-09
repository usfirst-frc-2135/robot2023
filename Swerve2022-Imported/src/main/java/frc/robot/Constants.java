
package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  /* Control Board */
  public static final int    kDriverPadPort    = 0;
  public static final int    kOperatorPadPort  = 1;

  public static final double kStickDeadband    = 0.15;
  public static final double kTriggerThreshold = 0.25;

  public static final class Ports
  {
    public static final String kCANivoreName     = "canivore1";

    public static final int    kCANID_DriveLF    = 10;
    public static final int    kCANID_TurnLF     = 11;
    public static final int    kCANID_CANCoderLF = 12;

    public static final int    kCANID_DriveRF    = 7;
    public static final int    kCANID_TurnRF     = 8;
    public static final int    kCANID_CANCoderRF = 9;

    public static final int    kCANID_DriveLR    = 4;
    public static final int    kCANID_TurnLR     = 5;
    public static final int    kCANID_CANCoderLR = 6;

    public static final int    kCANID_DriveRR    = 1;
    public static final int    kCANID_TurnRR     = 2;
    public static final int    kCANID_CANCoderRR = 3;

    public static final int    kCANID_Pigeon2    = 13;

    public static final int    kCANID_Intake     = 15;
    public static final int    kCANID_FloorConv  = 16;
    public static final int    kCANID_TowerConv  = 17;
    public static final int    kCANID_Shooter    = 18;

    // Digital I/Os
    public static final int    kDIO_CargoDetect  = 2;

    // PWM outputs
    public static final int    kPWM_Intake       = 1;
  }

  public static final class Falcon500
  {
    public static int          kMaxRPM               = 6380;             // free speed for Falcon 500 motor
    public static final double kEncoderCPR           = 2048;             // CPR is 2048 from Falcon 500 Manual
    public static final int    kTalonReqVersion      = ((22 * 256) + 0); // Talon version is 22.0
    public static final int    kPigeonReqVersion     = ((22 * 256) + 0); // Pigeon IMU version is 22.0

    // Input current limit settings
    public static final double kSupplyCurrentLimit   = 45.0;  // Default supply current limit (after trigger)
    public static final double kSupplyTriggerCurrent = 45.0;  // Trigger current that will cause limiting
    public static final double kSupplyTriggerTime    = 0.001; // Time duration of trigger that will causing limiting

    // Output current limit settings
    public static final double kStatorCurrentLimit   = 80.0;  // Default supply current limit (after trigger)
    public static final double kStatorTriggerCurrent = 80.0;  // Default trigger current that will cause limiting
    public static final double kStatorTriggerTime    = 0.001; // Default time duration of trigger that will causing limiting
  }

  public static final class SWConsts
  {
    // Swerve specs
    public static final double kWheelDiaMeters        = Units.inchesToMeters(4.0); // 4in (39.37 in/meter)
    public static final double kGearRatio             = 6.75;
    public static final double kEncoderMetersPerCount = (kWheelDiaMeters * Math.PI) / (Falcon500.kEncoderCPR) / kGearRatio;

    // Joystick tuning
    public static final double kDriveXScaling         = 1.0;           // 1.0 is no scaling
    public static final double kDriveYScaling         = 1.0;           // 1.0 is no scaling
    public static final double kQuickTurnScaling      = 0.5;           // Scale by 50% of full speed
    public static final double kSlowClimbScaling      = 0.3;           // Scale by 30% of full speed

    // Measured characterization
    public static final double ks                     = 0.65;          // Volts
    public static final double kv                     = 2.84;          // Volts / mps
    public static final double ka                     = 0.309;         // Volts / (mps^2)
    public static final double KvAngular              = 1.5;           // Volts / (rad/sec)
    public static final double KaAngular              = 0.3;           // Volts / (rad/sec^2)

    // Teleop driving controls
    public static final double kOpenLoopRamp          = 0.5;           // CTRE: full speed in 0.5 sec
    public static final double kClosedLoopRamp        = 0.0;           // CTRE: 0 is disabled
    public static final double kStopTolerance         = 0.05;          // Target position tolerance (< 5cm)

    // Limelight driving controls
    public static final double kTurnConstant          = 0.0;
    public static final double kTurnPidKp             = 0.005;
    public static final double kTurnPidKi             = 0.0;
    public static final double kTurnPidKd             = 0.0;
    public static final double kTurnMax               = 0.4;
    public static final double kThrottlePidKp         = 0.011;
    public static final double kThrottlePidKi         = 0.0;
    public static final double kThrottlePidKd         = 0.0;
    public static final double kThrottleMax           = 0.2;
    public static final double kThrottleShape         = 10.0;

    public static final double kTargetAngle           = 0.0;           // Optimal shooting angle
    public static final double kSetPointDistance      = 60.0;          // Optimal shooting distance
    public static final double kAngleThreshold        = 3.5;           // Degrees tolerance around optimal
    public static final double kDistThreshold         = 6.0;           // Inches tolerance around optimal
  }

  public static final class INConsts
  {
    // public static final int kIN8CANID = 6;
    public static final int    kArmSolenoid    = 0;

    public static final double kINAcquireSpeed = 0.6;
    public static final double kINExpelSpeed   = -0.6;

    public enum INMode
    {
      INTAKE_STOP,    // Stop intake motor
      INTAKE_ACQUIRE, // Acquire game pieces
      INTAKE_EXPEL,   // Expel game pieces
    }
  }

  public static final class FCConsts
  {
    public static final double kFCAcquireSpeed     = 1.0;
    public static final double kFCAcquireSpeedSlow = 0.2;
    public static final double kFCExpelSpeedFast   = -1.0;

    public enum FCMode
    {
      FCONVEYOR_STOP,       // Stop floor conveyor motor
      FCONVEYOR_ACQUIRE,    // Aquire game pieces
      FCONVEYOR_EXPEL,      // Expel game pieces
      FCONVEYOR_EXPEL_FAST, // Expel Fast
    }
  }

  public static final class TCConsts
  {
    public static final double kTCAcquireSpeed     = 1.0;
    public static final double kTCAcquireSpeedSlow = 0.2;
    public static final double kTCExpelSpeed       = -0.2;
    public static final double kTCExpelSpeedFast   = -1.0;

    public enum TCMode
    {
      TCONVEYOR_STOP,         // Conveyor stop
      TCONVEYOR_ACQUIRE,      // Conveyor moves game pieces to shooter
      TCONVEYOR_ACQUIRE_SLOW, // Conveyor moves during game piece intake
      TCONVEYOR_EXPEL,        // Conveyor moves game pieces to hopper
      TCONVEYOR_EXPEL_FAST,   // Conveyor moves game pieces to hopper
    }
  }

  public static final class SHConsts
  {
    public static final double                   kFlywheelGearRatio       = (18.0 / 12.0);
    public static final double                   kFlywheelCPR             = Falcon500.kEncoderCPR * kFlywheelGearRatio;

    public static final int                      kVelocityMeasWindow      = 1;
    public static final SensorVelocityMeasPeriod kVelocityMeasPeriod      = SensorVelocityMeasPeriod.Period_10Ms;
    public static final double                   kFlywheelPidKf           = 0.04775;
    public static final double                   kFlywheelPidKp           = 0.2;
    public static final double                   kFlywheelPidKi           = 0.0;
    public static final double                   kFlywheelPidKd           = 0.0;
    public static final double                   kFlywheelNeutralDeadband = 0.01;

    public static final double                   kFlywheelToleranceRPM    = 150.0;     // Tolerance band around target RPM
    public static final double                   kFlywheelLowerTargetRPM  = 1000.0;    // RPM for lower hub
    public static final double                   kFlywheelUpperTargetRPM  = 2150.0;    // RPM for upper hub
    public static final double                   kFlywheelPrimeRPM        = kFlywheelUpperTargetRPM; // RPM for shooter priming

    public static final double                   kReverseRPMThreshold     = 20.0;      // RPM threshold for allowing reverse
    public static final double                   kFlywheelReverseRPM      = -1000.0;   // RPM for reversing out game pieces

    public enum SHMode
    {
      SHOOTER_REVERSE,    // Shooter runs in reverse direction to handle jams
      SHOOTER_STOP,       // Shooter is stopped
      SHOOTER_PRIME,      // Shooter ramped to an initial speed before shooting
      SHOOTER_LOWERHUB,   // Shooter at speed for low hub
      SHOOTER_UPPERHUB,   // Shooter at speed for high hub
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
      VISION_OFF, VISION_ON, VISION_TOGGLE
    }

    public static final double kLLDistance1   = 48;    // distance from bumper in inches for first reference point
    public static final double kLLVertOffset1 = 0.42;  // LL y reading in degrees for first reference point
    public static final double kLLDistance2   = 60;    // distance from bumper in inches for second reference point
    public static final double kLLVertOffset2 = -4.85; // LL y reading in degrees for second reference point
  }

  public static final class LEDConsts
  {
    public static final int kCANDdleID = 0;

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

  public static final class SIMLLConsts
  {
    public static final double kFieldLength        = Units.feetToMeters(54.0);      // Field dimensions are 54ft x 27ft
    public static final double kFieldWidth         = Units.feetToMeters(27.0);
    public static final double kGoalPostionX       = kFieldLength / 2 - Units.feetToMeters(2.0); // Goal target on field
    public static final double kGoalPostionY       = kFieldWidth / 2;
    public static final double kGoalHeight         = Units.inchesToMeters(102.81);  // Upper hub height from floor
    public static final double kCameraPositionX    = Units.inchesToMeters(0.0);     // Camera position on robot (X, Y)
    public static final double kCameraPositionY    = Units.inchesToMeters(0.0);
    public static final double kCameraRotation     = Units.degreesToRadians(180.0); // Camera rotation on robot
    public static final double kCameraLensHeight   = Units.inchesToMeters(41.0);    // Camera lens height from floor
    public static final double kCameraLensBackTilt = Units.degreesToRadians(40.0);  // Camera backward tilt from normal
  }

  public static final class AUTOConstants
  {
    public static final String  kOneBallLimelight_path1    = "fenderToOffTarmac";
    public static final String  kOneBallLimelight_path2    = "shootingPosToOffTarmac";

    public static final String  kDrive_path                = "startToOffTarmac";

    public static final String  kDriveShoot_path1          = "startToShootingPos";
    public static final String  kDriveShoot_path2          = "shootingPosToOffTarmac";

    public static final String  kShootDriveShoot_path1     = "startToShootingPos";
    public static final String  kShootDriveShoot_path2     = "shootingPosToBall";
    public static final String  kShootDriveShoot_path3     = "ballToShootingPos";
    public static final String  kShootDriveShoot_path4     = "shootingPosToOffTarmac";

    public static final String  k3BallLeft_path1           = "startToShootingPos";
    public static final String  k3BallLeft_path2           = "shootingPosToBall";
    public static final String  k3BallLeft_path3           = "ballToShootingPos";
    public static final String  k3BallLeft_path4           = "shootingPosToLeftBall";
    public static final String  k3BallLeft_path5           = "leftBallToLeftShootingPos";

    public static final String  k3BallRight_path1          = "startToShootingPos";
    public static final String  k3BallRight_path2          = "shootingPosToBall";
    public static final String  k3BallRight_path3          = "ballToShootingPos";
    public static final String  k3BallRight_path4          = "shootingPosToRightBall";
    public static final String  k3BallRight_path5          = "rightBallToRightShootingPos";
    public static final String  k3BallRight_path6          = "shootingPosToOffTarmac";

    public static final String  k1BallLimelight_path1      = "fenderToOffTarmac";
    public static final String  k1BallLimelight_path2      = "shootingPosToOffTarmac";

    public static final String  k1Ball2OppLeft_path1       = "startToShootingPos";
    public static final String  k1Ball2OppLeft_path2       = "shootingPosToLeftOppBall1";
    public static final String  k1Ball2OppLeft_path3       = "leftOppBall1ToBall2";
    public static final String  k1Ball2OppLeft_path4       = "leftOppBall2ToShootingPos";

    public static final String  k1Ball1OppRight_path1      = "rightstarttoSP";
    public static final String  k1Ball1OppRight_path2      = "rightSPtoball";

    public static final boolean k_ShootOppBall             = true;

    public static final double  k_WaitTime1                = 0.0; // First wait timer - time to wait
    public static final double  k_WaitTime2                = 0.0; // Second wait timer - time to wait

    public static final String  path1                      = "forward39";
    public static final String  path2                      = "backward39";
    public static final String  path3                      = "rightAngleTurn";
    public static final String  kDriveLimelightShoot_path1 = "forward39";
    public static final String  kDriveLimelightShoot_path2 = "backward39";
    public static final String  kShoot_path                = "startToShootingPos";

    public enum AutoTimer
    {
      TIMER1,     // Select first auto wait timer for use
      TIMER2      // Select second auto wait timer for use
    };
  }

  ///// 1678 Constants /////
  // toggle constants between comp bot and practice bot ("epsilon")
  public static final boolean isComp = true;

  public static final class SwerveConstants
  {
    public static final boolean                                      invertGyro                  = false; // Always ensure Gyro is CCW+ CW-

    /* Swerve Constants - 0.427 m (x, y) */
    public static final double                                       trackWidth                  = Units.inchesToMeters(23.77);
    public static final double                                       wheelBase                   = Units.inchesToMeters(23.77);

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
    public static final boolean                                      driveMotorInvert            = false;
    public static final boolean                                      angleMotorInvert            = true;

    /* Angle Encoder Invert */
    public static final boolean                                      canCoderInvert              = false;

    /* Controller Invert */
    public static final boolean                                      invertYAxis                 = false;
    public static final boolean                                      invertRAxis                 = false;
    public static final boolean                                      invertXAxis                 = false;

    /*** MODULE SPECIFIC CONSTANTS ***/

    /* Front Left Module - Module 0 */
    public static final class Mod0
    {
      public static final double epsilonAngleOffset = 239.06;
      public static final double compAngleOffset    = 187.646;

      public static SwerveModuleConstants SwerveModuleConstants( )
      {
        return new SwerveModuleConstants(Ports.kCANID_DriveLF, Ports.kCANID_TurnLF, Ports.kCANID_CANCoderLF,
            isComp ? compAngleOffset : epsilonAngleOffset);
      }
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1
    {
      public static final double epsilonAngleOffset = 339.96;
      public static final double compAngleOffset    = 2.461;

      public static SwerveModuleConstants SwerveModuleConstants( )
      {
        return new SwerveModuleConstants(Ports.kCANID_DriveRF, Ports.kCANID_TurnRF, Ports.kCANID_CANCoderRF,
            isComp ? compAngleOffset : epsilonAngleOffset);
      }
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2
    {
      public static final double epsilonAngleOffset = 317.20;
      public static final double compAngleOffset    = 60.029;

      public static SwerveModuleConstants SwerveModuleConstants( )
      {
        return new SwerveModuleConstants(Ports.kCANID_DriveLR, Ports.kCANID_TurnLR, Ports.kCANID_CANCoderLR,
            isComp ? compAngleOffset : epsilonAngleOffset);
      }
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3
    {
      public static final double epsilonAngleOffset = 311.22;
      public static final double compAngleOffset    = 195.117;

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

  /*** SUBSYSTEM CONSTANTS ***/

  // Timeout constants
  public static final int kLongCANTimeoutMs = 100;
  public static final int kCANTimeoutMs     = 10;

  ///// 1678 Constants /////
}
