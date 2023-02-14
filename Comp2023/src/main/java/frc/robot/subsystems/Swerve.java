//
// Swerve subystem - handles swerve driving and path following
//
package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SWConsts;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.team1678.frc2022.drivers.Pigeon;
import frc.robot.team1678.frc2022.drivers.SwerveModule;
import frc.robot.team254.lib.util.TimeDelayedBoolean;

//
// Swerve subsystem class
//
public class Swerve extends SubsystemBase
{
  // Member objects
  private SwerveModule[ ]          m_swerveMods          = new SwerveModule[ ]
  {
      new SwerveModule(0, Constants.SwerveConstants.Mod0.SwerveModuleConstants( )),
      new SwerveModule(1, Constants.SwerveConstants.Mod1.SwerveModuleConstants( )),
      new SwerveModule(2, Constants.SwerveConstants.Mod2.SwerveModuleConstants( )),
      new SwerveModule(3, Constants.SwerveConstants.Mod3.SwerveModuleConstants( ))
  };

  // Odometery and telemetry
  private Pigeon                   m_pigeon              = new Pigeon(Ports.kCANID_Pigeon2);
  private SwerveDriveOdometry      m_swerveOdometry;

  public SwerveDrivePoseEstimator  m_poseEstimator       = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics,
      m_pigeon.getYaw( ).getWPIRotation2d( ), getPositions( ), new Pose2d( ));

  // PID objects
  private ProfiledPIDController    m_snapPIDController   = new ProfiledPIDController( // 
      Constants.SnapConstants.kP, //
      Constants.SnapConstants.kI, //
      Constants.SnapConstants.kD, //
      Constants.SnapConstants.kThetaControllerConstraints);
  private PIDController            m_visionPIDController =
      new PIDController(Constants.VisionAlignConstants.kP, Constants.VisionAlignConstants.kI, Constants.VisionAlignConstants.kD);

  // Holonomic Drive Controller objects
  private HolonomicDriveController m_holonomicController;
  private PathPlannerTrajectory    m_trajectory;
  private Timer                    m_trajTimer           = new Timer( );

  // Module variables
  private PeriodicIO               m_periodicIO          = new PeriodicIO( );
  private boolean                  m_isSnapping;
  private double                   m_limelightVisionAlignGoal;
  private double                   m_visionAlignAdjustment;

  // Lock Swerve wheels
  private boolean                  m_locked              = false;

  // Path following
  private int                      m_pathDebug           = 1;    // Debug flag to disable extra ramsete logging calls

  // Limelight drive
  private double                   m_turnConstant        = SWConsts.kTurnConstant;
  private double                   m_turnPidKp           = SWConsts.kTurnPidKp;
  private double                   m_turnPidKi           = SWConsts.kTurnPidKi;
  private double                   m_turnPidKd           = SWConsts.kTurnPidKd;
  private double                   m_turnMax             = SWConsts.kTurnMax;
  private double                   m_throttlePidKp       = SWConsts.kThrottlePidKp;
  private double                   m_throttlePidKi       = SWConsts.kThrottlePidKi;
  private double                   m_throttlePidKd       = SWConsts.kThrottlePidKd;
  private double                   m_throttleMax         = SWConsts.kThrottleMax;
  private double                   m_throttleShape       = SWConsts.kThrottleShape;

  private double                   m_targetAngle         = SWConsts.kTargetAngle; // Optimal shooting angle
  private double                   m_setPointDistance    = SWConsts.kSetPointDistance; // Optimal shooting distance
  private double                   m_angleThreshold      = SWConsts.kAngleThreshold; // Tolerance around optimal
  private double                   m_distThreshold       = SWConsts.kDistThreshold;// Tolerance around optimal

  // DriveWithLimelight pid controller objects
  private int                      m_limelightDebug      = 0; // Debug flag to disable extra limelight logging calls
  private PIDController            m_turnPid             = new PIDController(0.0, 0.0, 0.0);
  private PIDController            m_throttlePid         = new PIDController(0.0, 0.0, 0.0);
  private double                   m_limelightDistance;

  // define theta controller for robot heading
  PIDController                    xController           = new PIDController(1, 0, 0);
  PIDController                    yController           = new PIDController(1, 0, 0);
  ProfiledPIDController            thetaController       = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0,
      0, Constants.AutoConstants.kThetaControllerConstraints);

  public Swerve( )
  {
    setName("Swerve");
    setSubsystem("Swerve");

    zeroGyro( );

    m_swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, m_pigeon.getYaw( ).getWPIRotation2d( ),
        new SwerveModulePosition[ ]
        {
            m_swerveMods[0].getPosition( ), //
            m_swerveMods[1].getPosition( ), //
            m_swerveMods[2].getPosition( ), //
            m_swerveMods[3].getPosition( )
        });

    m_snapPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_visionPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_visionPIDController.setTolerance(0.0);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    resetOdometry(new Pose2d( ));
    resetAnglesToAbsolute( );

    initSmartDashboard( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    updateSwerveOdometry( );
    readPeriodicInputs( );
    updateSmartDashboard( );
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run when in simulation
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {

  }

  public void faultDump( )
  {

  }

  private void initSmartDashboard( )
  {}

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Periodic helper methods
  //
  private void updateSmartDashboard( )
  {
    SmartDashboard.putNumber("SWMod: 0 - Speed", m_swerveMods[0].getState( ).speedMetersPerSecond);
    SmartDashboard.putNumber("SWMod: 0 - Angle", m_swerveMods[0].getState( ).angle.getDegrees( ));
    SmartDashboard.putNumber("SWMod: 0 - Dist", m_swerveMods[0].getPosition( ).distanceMeters);
    SmartDashboard.putNumber("SWMod: 1 - Speed", m_swerveMods[1].getState( ).speedMetersPerSecond);
    SmartDashboard.putNumber("SWMod: 1 - Angle", m_swerveMods[1].getState( ).angle.getDegrees( ));
    SmartDashboard.putNumber("SWMod: 1 - Dist", m_swerveMods[1].getPosition( ).distanceMeters);
    SmartDashboard.putNumber("SWMod: 2 - Speed", m_swerveMods[2].getState( ).speedMetersPerSecond);
    SmartDashboard.putNumber("SWMod: 2 - Angle", m_swerveMods[2].getState( ).angle.getDegrees( ));
    SmartDashboard.putNumber("SWMod: 2 - Dist", m_swerveMods[2].getPosition( ).distanceMeters);
    SmartDashboard.putNumber("SWMod: 3 - Speed", m_swerveMods[3].getState( ).speedMetersPerSecond);
    SmartDashboard.putNumber("SWMod: 3 - Angle", m_swerveMods[3].getState( ).angle.getDegrees( ));
    SmartDashboard.putNumber("SWMod: 3 - Dist", m_swerveMods[3].getPosition( ).distanceMeters);

    SmartDashboard.putNumber("SW: pose_x", m_periodicIO.odometry_pose_x);
    SmartDashboard.putNumber("SW: pose_y", m_periodicIO.odometry_pose_y);
    SmartDashboard.putNumber("SW: pose_rot", m_periodicIO.odometry_pose_rot);

    SmartDashboard.putNumber("SW: swerve-hdg", m_periodicIO.swerve_heading);
    SmartDashboard.putNumber("SW: pitch", m_periodicIO.robot_pitch);
    SmartDashboard.putNumber("SW: roll", m_periodicIO.robot_roll);

    SmartDashboard.putNumber("SW: vision", m_periodicIO.vision_align_target_angle);
    SmartDashboard.putNumber("SW: snap", m_periodicIO.snap_target);

    // //Setting the field to the Pose2d specified by the limelight
    RobotContainer.getInstance( ).m_field2d.setRobotPose(m_poseEstimator.getEstimatedPosition( ));
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Teleop driving mode
  //

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Limelight driving mode
  //
  public void driveWithLimelightInit(boolean m_endAtTarget)
  {
    // get pid values from dashboard
    m_turnConstant = SmartDashboard.getNumber("DTL_turnConstant", m_turnConstant);
    m_turnPidKp = SmartDashboard.getNumber("DTL_turnPidKp", m_turnPidKp);
    m_turnPidKi = SmartDashboard.getNumber("DTL_turnPidKi", m_turnPidKi);
    m_turnPidKd = SmartDashboard.getNumber("DTL_turnPidKd", m_turnPidKd);
    m_turnMax = SmartDashboard.getNumber("DTL_turnMax", m_turnMax);

    m_throttlePidKp = SmartDashboard.getNumber("DTL_throttlePidKp", m_throttlePidKp);
    m_throttlePidKi = SmartDashboard.getNumber("DTL_throttlePidKi", m_throttlePidKi);
    m_throttlePidKd = SmartDashboard.getNumber("DTL_throttlePidKd", m_throttlePidKd);
    m_throttleMax = SmartDashboard.getNumber("DTL_throttleMax", m_throttleMax);
    m_throttleShape = SmartDashboard.getNumber("DTL_throttleShape", m_throttleShape);

    m_targetAngle = SmartDashboard.getNumber("DTL_targetAngle", m_targetAngle);
    m_setPointDistance = SmartDashboard.getNumber("DTL_setPointDistance", m_setPointDistance);
    m_angleThreshold = SmartDashboard.getNumber("DTL_angleThreshold", m_angleThreshold);
    m_distThreshold = SmartDashboard.getNumber("DTL_distThreshold", m_distThreshold);

    // load in Pid constants to controller
    m_turnPid = new PIDController(m_turnPidKp, m_turnPidKi, m_turnPidKd);
    m_throttlePid = new PIDController(m_throttlePidKp, m_throttlePidKi, m_throttlePidKd);

    RobotContainer rc = RobotContainer.getInstance( );
    rc.m_vision.m_tyfilter.reset( );
    rc.m_vision.m_tvfilter.reset( );
    rc.m_vision.syncStateFromDashboard( );
  }

  public void driveWithLimelightExecute( )
  {
    RobotContainer rc = RobotContainer.getInstance( );
    boolean tv = rc.m_vision.getTargetValid( );
    double tx = rc.m_vision.getHorizOffsetDeg( );
    double ty = rc.m_vision.getVertOffsetDeg( );

    if (!tv)
    {
      drive(new Translation2d(0.0, 0.0), 0.0, false, true);
      if (m_limelightDebug >= 1)
        DataLogManager.log(getSubsystem( ) + ": DTL TV-FALSE - SIT STILL");

      return;
    }

    // get turn value - just horizontal offset from target
    double turnOutput = -m_turnPid.calculate(tx, m_targetAngle);

    if (turnOutput > 0)
      turnOutput = turnOutput + m_turnConstant;
    else if (turnOutput < 0)
      turnOutput = turnOutput - m_turnConstant;

    // get throttle value
    m_limelightDistance = RobotContainer.getInstance( ).m_vision.getDistLimelight( );

    double throttleDistance = m_throttlePid.calculate(m_limelightDistance, m_setPointDistance);
    double throttleOutput = throttleDistance * Math.pow(Math.cos(turnOutput * Math.PI / 180), m_throttleShape);

    // put turn and throttle outputs on the dashboard
    SmartDashboard.putNumber("DTL_turnOutput", turnOutput);
    SmartDashboard.putNumber("DTL_throttleOutput", throttleOutput);
    SmartDashboard.putNumber("DTL_limeLightDist", m_limelightDistance);

    // cap max turn and throttle output
    turnOutput = MathUtil.clamp(turnOutput, -m_turnMax, m_turnMax);
    throttleOutput = MathUtil.clamp(throttleOutput, -m_throttleMax, m_throttleMax);

    // put turn and throttle outputs on the dashboard
    SmartDashboard.putNumber("DTL_turnClamped", turnOutput);
    SmartDashboard.putNumber("DTL_throttleClamped", throttleOutput);

    Translation2d llTranslation = new Translation2d(throttleOutput, 0);
    drive(llTranslation, turnOutput, false, true);

    if (m_limelightDebug >= 1)
      DataLogManager.log(getSubsystem( )
      // @formatter:off
        + ": DTL tv: " + tv 
        + " tx: "      + String.format("%.1f", tx)
        + " ty: "      + String.format("%.1f", ty)
        + " lldist: "  + String.format("%.1f", m_limelightDistance)
        + " distErr: " + String.format("%.1f", Math.abs(m_setPointDistance - m_limelightDistance))
        // + " stopped: " + driveIsStopped( )
        + " trnOut: "  + String.format("%.2f", turnOutput)
        + " thrOut: "  + String.format("%.2f", throttleOutput)
        // @formatter:on
      );
  }

  public boolean driveWithLimelightIsFinished( )
  {
    RobotContainer rc = RobotContainer.getInstance( );
    boolean tv = rc.m_vision.getTargetValid( );
    double tx = rc.m_vision.getHorizOffsetDeg( );

    return (tv && ((Math.abs(tx)) <= m_angleThreshold) && (Math.abs(m_setPointDistance - m_limelightDistance) <= m_distThreshold)
    // TODO: add back in
    // && driveIsStopped( )
    );
  }

  public void driveWithLimelightEnd( )
  {
    drive(new Translation2d(0.0, 0.0), 0.0, false, true);

    // RobotContainer.getInstance( ).m_led.setLLColor(LEDColor.LEDCOLOR_OFF);
  }

  public boolean isLimelightValid(double horizAngleRange, double distRange)
  {
    // check whether target is valid
    // check whether the limelight tx and ty is within a certain tolerance
    // check whether distance is within a certain tolerance
    RobotContainer rc = RobotContainer.getInstance( );
    boolean tv = rc.m_vision.getTargetValid( );
    double tx = rc.m_vision.getHorizOffsetDeg( );
    double ty = rc.m_vision.getVertOffsetDeg( );
    m_limelightDistance = rc.m_vision.getDistLimelight( );

    boolean sanityCheck =
        tv && (Math.abs(tx) <= horizAngleRange) && (Math.abs(m_setPointDistance - m_limelightDistance) <= distRange);
    // && (fabs(ty) <= vertAngleRange)

    DataLogManager.log(getSubsystem( ) + ": DTL tv: " + tv //
        + " tx: " + tx //
        + " ty: " + ty //
        + " lldist: " + m_limelightDistance //
        + " distErr: " + Math.abs(m_setPointDistance - m_limelightDistance) //
        + " sanityCheck: " + ((sanityCheck) ? "PASSED" : "FAILED") //
    );

    return sanityCheck;
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Autonomous mode - Holonomic path follower
  //
  public void driveWithPathFollowerInit(PathPlannerTrajectory trajectory, boolean useInitialPose)
  {
    m_holonomicController = new HolonomicDriveController(xController, yController, thetaController);

    m_trajectory = trajectory;

    if (!RobotBase.isReal( ))
    {
      RobotContainer.getInstance( ).m_field2d.getObject("trajectory").setTrajectory(m_trajectory);
    }

    List<Trajectory.State> trajStates = new ArrayList<Trajectory.State>( );
    trajStates = m_trajectory.getStates( );
    DataLogManager.log(getSubsystem( ) + ": DTR states: " + trajStates.size( ) + " dur: " + m_trajectory.getTotalTimeSeconds( ));

    // This initializes the odometry (where we are)
    if (useInitialPose)
      resetOdometry(m_trajectory.getInitialHolonomicPose( ));

    m_trajTimer.reset( );
    m_trajTimer.start( );
  }

  public void driveWithPathFollowerExecute( )
  {
    Trajectory.State trajState = m_trajectory.sample(m_trajTimer.get( ));
    Pose2d currentPose = getPose( );

    ChassisSpeeds targetChassisSpeeds = m_holonomicController.calculate(currentPose, trajState,
        m_trajectory.getEndState( ).holonomicRotation/* trajState.poseMeters.getRotation( ) */); //TODO: find out what's wrong with getting desired rotation

    // Convert to module states
    SwerveModuleState[ ] moduleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(targetChassisSpeeds);

    double targetfrontLeft = (moduleStates[0].speedMetersPerSecond); //TODO: Add MPS ??
    double targetfrontRight = (moduleStates[1].speedMetersPerSecond); //TODO: Add MPS ??
    double targetbackLeft = (moduleStates[2].speedMetersPerSecond);
    double targetbackRight = (moduleStates[3].speedMetersPerSecond);

    double currentfrontLeft = m_swerveMods[0].getState( ).speedMetersPerSecond;
    double currentfrontRight = m_swerveMods[1].getState( ).speedMetersPerSecond;
    double currentbackLeft = m_swerveMods[2].getState( ).speedMetersPerSecond;
    double currentbackRight = m_swerveMods[3].getState( ).speedMetersPerSecond;

    double targetTrajX = trajState.poseMeters.getX( );
    double targetTrajY = trajState.poseMeters.getY( );
    double currentTrajX = currentPose.getX( );
    double currentTrajY = currentPose.getY( );

    double targetHeading =
        m_trajectory.getEndState( ).holonomicRotation.getDegrees( )/* trajState.poseMeters.getRotation( ).getDegrees( ) */; ///Maybe get in radians?
    double currentHeading = currentPose.getRotation( ).getDegrees( ); ///Maybe get in radians?

    //TODO: feedWatchDog

    setModuleStates(moduleStates);

    if (m_pathDebug >= 1)
    {
      DataLogManager.log(getSubsystem( )
     // @formatter:off
            + ": DTR time: "     + String.format("%.3f", m_trajTimer.get( ))
                + " curXYR: "    + String.format("%.2f", currentTrajX) 
                  + " "          + String.format("%.2f", currentTrajY) 
                  + " "          + String.format("%.1f", currentHeading)
                + " targXYR: "   + String.format("%.2f", targetTrajX) 
                  + " "          + String.format("%.2f", targetTrajY) 
                  + " "          + String.format("%.1f", targetHeading)
                + " chasXYO: "   + String.format("%.1f", targetChassisSpeeds.vxMetersPerSecond) 
                  + " "          + String.format("%.1f", targetChassisSpeeds.vyMetersPerSecond)
                  + " "          + String.format("%.1f", Units.radiansToDegrees(targetChassisSpeeds.omegaRadiansPerSecond))
                + " targVel: "   + String.format("%.1f", moduleStates[0].speedMetersPerSecond) 
                  + " "          + String.format("%.1f", moduleStates[1].speedMetersPerSecond)
                  + " "          + String.format("%.1f", moduleStates[2].speedMetersPerSecond) 
                  + " "          + String.format("%.1f", moduleStates[3].speedMetersPerSecond)
                + " curVel: "    + String.format("%.2f", currentfrontLeft) 
                  + " "          + String.format("%.2f", currentfrontRight)
                  + " "          + String.format("%.2f", currentbackLeft)
                  + " "          + String.format("%.2f", currentbackRight)
                + " errorXYR: "    + String.format("%.2f", targetTrajX-currentTrajX) 
                  + " "          + String.format("%.2f", targetTrajY-currentTrajY)
                  + " "          + String.format("%.2f", targetHeading-currentHeading)); 
        // @formatter:on
    }

    if (m_pathDebug >= 2)
    {
      // target velocity and its error
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_targetVelFrontLeft", targetfrontLeft);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_targetVelFrontRight", targetfrontRight);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_targetVelBackLeft", targetbackLeft);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_targetVelBackRight", targetbackRight);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_currentVelFrontLeft", currentfrontLeft);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_currentVelFrontRight", currentfrontRight);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_currentVelBackLeft", currentbackLeft);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_currentVelBackRight", currentbackRight);

      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_velErrorFrontLeft", targetfrontLeft - currentfrontLeft);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_velErrorFrontRight", targetfrontRight - currentfrontRight);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_velErrorBackLeft", targetbackLeft - currentbackLeft);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_velErrorBackRight", targetbackRight - currentbackRight);

      // target distance and its error
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_currentTrajX", targetTrajX);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_currentTrajY", targetTrajY);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_targetTrajX", currentTrajX);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_targetTrajY", currentTrajY);

      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_trajErrorX", trajState.poseMeters.relativeTo(currentPose).getX( ));
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_trajErrorY", trajState.poseMeters.relativeTo(currentPose).getY( ));

      // target heading and its error
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_targetHeading", targetHeading);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_currentHeading", currentHeading);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_headingError",
          trajState.poseMeters.relativeTo(currentPose).getRotation( ).getDegrees( ));
    }
  }

  public boolean driveWithPathFollowerIsFinished( )
  {
    if (m_trajTimer.get( ) == 0)
      return false;

    if (m_trajTimer.get( ) >= 15.0)
    {
      DataLogManager.log(getSubsystem( ) + ": path follower timeout!");
      return true;
    }

    return (m_trajTimer.hasElapsed(m_trajectory.getTotalTimeSeconds( ) + 0.25));
  }

  public void driveWithPathFollowerEnd( )
  {
    m_trajTimer.stop( );
  }

  //// 1678 Swerve //////////////////////////////////////////////////////////////

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
  {
    if (m_isSnapping)
    {
      if (Math.abs(rotation) == 0.0)
      {
        maybeStopSnap(false);
        rotation = calculateSnapValue( );
      }
      else
      {
        maybeStopSnap(true);
      }
    }

    SwerveModuleState[ ] swerveModuleStates = null;
    if (m_locked)
    {
      swerveModuleStates = new SwerveModuleState[ ]
      {
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)), //
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)), //
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)), //
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
      };
    }
    else
    {
      swerveModuleStates =
          Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX( ), translation.getY( ), rotation,
                  m_pigeon.getYaw( ).getWPIRotation2d( ))
              : new ChassisSpeeds(translation.getX( ), translation.getY( ), rotation));
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : m_swerveMods)
    {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  //
  // Snap to a direction
  //  
  public double calculateSnapValue( )
  {
    return m_snapPIDController.calculate(m_pigeon.getYaw( ).getRadians( ));
  }

  public void startSnap(double snapAngle)
  {
    m_snapPIDController.reset(m_pigeon.getYaw( ).getRadians( ));
    m_snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
    m_isSnapping = true;
  }

  TimeDelayedBoolean delayedBoolean = new TimeDelayedBoolean( );

  private boolean snapComplete( )
  {
    double error = m_snapPIDController.getGoal( ).position - m_pigeon.getYaw( ).getRadians( );
    return delayedBoolean.update(Math.abs(error) < Math.toRadians(Constants.SnapConstants.kEpsilon),
        Constants.SnapConstants.kTimeout);
  }

  public void maybeStopSnap(boolean force)
  {
    if (!m_isSnapping)
    {
      return;
    }
    if (force || snapComplete( ))
    {
      m_isSnapping = false;
      m_snapPIDController.reset(m_pigeon.getYaw( ).getRadians( ));
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[ ] desiredStates)
  {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : m_swerveMods)
    {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
      SmartDashboard.putNumber("mod " + mod.moduleNumber + " desired speed",
          desiredStates[mod.moduleNumber].speedMetersPerSecond);
      SmartDashboard.putNumber("mod " + mod.moduleNumber + " desired angle",
          MathUtil.inputModulus(desiredStates[mod.moduleNumber].angle.getDegrees( ), 0, 180));
    }
  }

  // Getters and setters

  public boolean getLocked( )
  {
    return m_locked;
  }

  public void setLocked(boolean lock)
  {
    m_locked = lock;
  }

  public Pose2d getPose( )
  {
    return m_swerveOdometry.getPoseMeters( );
  }

  public void resetOdometry(Pose2d pose)
  {
    m_swerveOdometry.resetPosition(m_pigeon.getYaw( ).getWPIRotation2d( ), getPositions( ), pose);
    zeroGyro(pose.getRotation( ).getDegrees( ));
  }

  public void resetAnglesToAbsolute( )
  {
    for (SwerveModule mod : m_swerveMods)
    {
      mod.resetToAbsolute( );
    }
  }

  public SwerveModuleState[ ] getStates( )
  {
    SwerveModuleState[ ] states = new SwerveModuleState[4];
    for (SwerveModule mod : m_swerveMods)
    {
      states[mod.moduleNumber] = mod.getState( );
      SmartDashboard.putNumber("mod " + mod.moduleNumber + " current speed", states[mod.moduleNumber].speedMetersPerSecond);
      SmartDashboard.putNumber("mod " + mod.moduleNumber + " current angle",
          MathUtil.inputModulus(states[mod.moduleNumber].angle.getDegrees( ), 0, 180));
    }
    return states;
  }

  public SwerveModulePosition[ ] getPositions( )
  {
    SwerveModulePosition[ ] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : m_swerveMods)
    {
      positions[mod.moduleNumber] = mod.getPosition( );
      SmartDashboard.putNumber("mod " + mod.moduleNumber + " current distance", positions[mod.moduleNumber].distanceMeters);
      SmartDashboard.putNumber("mod " + mod.moduleNumber + " current angle",
          MathUtil.inputModulus(positions[mod.moduleNumber].angle.getDegrees( ), 0, 180));
    }
    return positions;
  }

  public void setAnglePIDValues(double kP, double kI, double kD)
  {
    for (SwerveModule swerveModule : m_swerveMods)
    {
      swerveModule.updateAnglePID(kP, kI, kD);
    }
  }

  public double[ ] getAnglePIDValues(int index)
  {
    return m_swerveMods[index].getAnglePIDValues( );
  }

  public void setVisionAlignPIDValues(double kP, double kI, double kD)
  {
    m_visionPIDController.setPID(kP, kI, kD);
  }

  public double[ ] getVisionAlignPIDValues( )
  {
    return new double[ ]
    {
        m_visionPIDController.getP( ), m_visionPIDController.getI( ), m_visionPIDController.getD( )
    };
  }

  public void zeroGyro( )
  {
    zeroGyro(0.0);
  }

  public void zeroGyro(double reset)
  {
    m_pigeon.setYaw(reset);
    m_visionPIDController.reset( );
  }

  public void updateSwerveOdometry( )
  {
    m_swerveOdometry.update(m_pigeon.getYaw( ).getWPIRotation2d( ), getPositions( ));
    m_poseEstimator.updateWithTime(Timer.getFPGATimestamp( ), m_pigeon.getYaw( ).getWPIRotation2d( ), getPositions( ));

    if (RobotContainer.getInstance( ).m_vision.addVisionMeasurement( ))
    {
      Pose2d botPose2d = RobotContainer.getInstance( ).m_vision.getBotPose2d( );
      double latency = RobotContainer.getInstance( ).m_vision.getTargetLatency( );

      //Adding a position specified by the limelight to the estimator at the time that the pose was generated 
      m_poseEstimator.addVisionMeasurement(botPose2d, Timer.getFPGATimestamp( ) - (0.001 * latency));
    }
  }

  public void readPeriodicInputs( )
  {
    m_periodicIO.odometry_pose_x = m_swerveOdometry.getPoseMeters( ).getX( );
    m_periodicIO.odometry_pose_y = m_swerveOdometry.getPoseMeters( ).getY( );
    m_periodicIO.odometry_pose_rot = m_swerveOdometry.getPoseMeters( ).getRotation( ).getDegrees( );

    m_periodicIO.swerve_heading = MathUtil.inputModulus(m_pigeon.getYaw( ).getDegrees( ), 0, 360);
    m_periodicIO.robot_pitch = m_pigeon.getUnadjustedPitch( ).getDegrees( );
    m_periodicIO.robot_roll = m_pigeon.getRoll( ).getDegrees( );

    m_periodicIO.vision_align_target_angle = Math.toDegrees(m_limelightVisionAlignGoal);
    m_periodicIO.snap_target = Math.toDegrees(m_snapPIDController.getGoal( ).position);
  }

  public class PeriodicIO
  {
    // inputs
    public double odometry_pose_x;
    public double odometry_pose_y;
    public double odometry_pose_rot;

    public double swerve_heading;
    public double robot_pitch;
    public double robot_roll;
    public double vision_align_target_angle;

    // outputs
    public double snap_target;
  }

  //// 1678 Swerve //////////////////////////////////////////////////////////////

}
