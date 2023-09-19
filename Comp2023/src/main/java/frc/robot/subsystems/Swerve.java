//
// Swerve subystem - handles swerve driving and path following
//
package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LLConsts;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SWConsts;
import frc.robot.Constants.SnapConstants;
import frc.robot.RobotContainer;
import frc.robot.lib.util.PigeonIMU;
import frc.robot.lib.util.SwerveModule;

//
// Swerve subsystem class
//
public class Swerve extends SubsystemBase
{
  // Member objects
  private SwerveModule[ ]          m_swerveMods        = new SwerveModule[ ]
  {
      new SwerveModule(0, SWConsts.Mod0.SwerveModuleConstants( )), new SwerveModule(1, SWConsts.Mod1.SwerveModuleConstants( )),
      new SwerveModule(2, SWConsts.Mod2.SwerveModuleConstants( )), new SwerveModule(3, SWConsts.Mod3.SwerveModuleConstants( ))
  };

  // Odometery and telemetry
  private final PigeonIMU          m_pigeon            = new PigeonIMU(Ports.kCANID_Pigeon2, Ports.kCANCarnivore);

  private SwerveDrivePoseEstimator m_poseEstimator     =
      new SwerveDrivePoseEstimator(SWConsts.swerveKinematics, m_pigeon.getYaw( ), getPositions( ), new Pose2d( ));
  private Field2d                  m_field             = new Field2d( );

  // PID objects
  private ProfiledPIDController    m_snapPIDController = new ProfiledPIDController( // 
      SnapConstants.kP, //
      SnapConstants.kI, //
      SnapConstants.kD, //
      SnapConstants.kThetaControllerConstraints);

  // Holonomic Drive Controller objects
  private HolonomicDriveController m_holonomicController;
  private PathPlannerTrajectory    m_trajectory;
  private Timer                    m_trajTimer         = new Timer( );
  private boolean                  m_isTeleported      = true;

  // Module variables
  private PeriodicIO               m_periodicIO        = new PeriodicIO( );
  private boolean                  m_isSnapping;
  // private double                   m_limelightVisionAlignGoal;
  private double                   m_pitch;

  // Lock Swerve wheels
  private boolean                  m_locked            = false;

  // Path following
  private int                      m_pathDebug         = 1;     // Debug flag to disable extra ramsete logging calls
  private boolean                  m_swerveDebug       = false; // Debug flag to disable extra ramsete logging calls

  // Limelight drive
  private double                   m_turnConstant      = LLConsts.kTurnConstant;
  private double                   m_turnPidKp         = LLConsts.kTurnPidKp;
  private double                   m_turnPidKi         = LLConsts.kTurnPidKi;
  private double                   m_turnPidKd         = LLConsts.kTurnPidKd;
  private double                   m_turnMax           = LLConsts.kTurnMax;
  private double                   m_throttlePidKp     = LLConsts.kThrottlePidKp;
  private double                   m_throttlePidKi     = LLConsts.kThrottlePidKi;
  private double                   m_throttlePidKd     = LLConsts.kThrottlePidKd;
  private double                   m_throttleMax       = LLConsts.kThrottleMax;
  private double                   m_throttleShape     = LLConsts.kThrottleShape;

  private double                   m_targetAngle       = LLConsts.kTargetAngle;      // Optimal shooting angle
  private double                   m_setPointDistance  = LLConsts.kSetPointDistance; // Optimal shooting distance
  private double                   m_angleThreshold    = LLConsts.kAngleThreshold;   // Tolerance around optimal
  private double                   m_distThreshold     = LLConsts.kDistThreshold;    // Tolerance around optimal

  // DriveWithLimelight pid controller objects
  private int                      m_limelightDebug    = 0; // Debug flag to disable extra limelight logging calls
  private PIDController            m_turnPid           = new PIDController(0.0, 0.0, 0.0);
  private PIDController            m_throttlePid       = new PIDController(0.0, 0.0, 0.0);
  private double                   m_limelightDistance;

  // define theta controller for robot heading
  PIDController                    m_xController       = new PIDController(1, 0, 0);
  PIDController                    m_yController       = new PIDController(1, 0, 0);
  ProfiledPIDController            m_thetaController   =
      new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  public Swerve( )
  {
    setName("Swerve");
    setSubsystem("Swerve");

    zeroGyro( );

    m_snapPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

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
    DataLogManager.log(getSubsystem( ) + ": Subsystem initialized!");
  }

  public void faultDump( )
  {}

  private void initSmartDashboard( )
  {
    // For future work to set up Shuffleboard layout from code
    // ShuffleboardTab swTab = Shuffleboard.getTab("Swerve");
    // swTab.add("SWM0_Velocity", m_swerveMods[0].getState( ).speedMetersPerSecond).withPosition(0, 0).withSize(2, 1);

    SmartDashboard.putData("Field", m_field);
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Periodic helper methods
  //
  public class PeriodicIO
  {
    // inputs
    public double odometry_pose_x;
    public double odometry_pose_y;
    public double odometry_pose_rot;

    public double swerve_heading;
    public double robot_pitch;
    public double robot_roll;
    // public double vision_align_target_angle;

    // outputs
    public double snap_target;
  }

  public void readPeriodicInputs( )
  {
    Pose2d position = getPose( );

    m_periodicIO.odometry_pose_x = position.getX( );
    m_periodicIO.odometry_pose_y = position.getY( );
    m_periodicIO.odometry_pose_rot = position.getRotation( ).getDegrees( );

    m_periodicIO.swerve_heading = MathUtil.inputModulus(m_pigeon.getYaw( ).getDegrees( ), 0, 360);
    m_periodicIO.robot_pitch = m_pigeon.getPitch( ).getDegrees( );
    m_periodicIO.robot_roll = m_pigeon.getRoll( ).getDegrees( );

    // m_periodicIO.vision_align_target_angle = Math.toDegrees(m_limelightVisionAlignGoal);
    m_periodicIO.snap_target = Math.toDegrees(m_snapPIDController.getGoal( ).position);
  }

  private void updateSmartDashboard( )
  {
    if (m_swerveDebug)
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
    }

    SmartDashboard.putNumber("SW: pose_x", m_periodicIO.odometry_pose_x);
    SmartDashboard.putNumber("SW: pose_y", m_periodicIO.odometry_pose_y);
    SmartDashboard.putNumber("SW: pose_rot", m_periodicIO.odometry_pose_rot);

    SmartDashboard.putNumber("SW: swerve-hdg", m_periodicIO.swerve_heading);
    SmartDashboard.putNumber("SW: pitch", m_periodicIO.robot_pitch);
    SmartDashboard.putNumber("SW: roll", m_periodicIO.robot_roll);

    // SmartDashboard.putNumber("SW: vision", m_periodicIO.vision_align_target_angle);
    SmartDashboard.putNumber("SW: snap", m_periodicIO.snap_target);

    m_field.setRobotPose(getPose( ));
  }

  public void updateSwerveOdometry( )
  {
    m_poseEstimator.updateWithTime(Timer.getFPGATimestamp( ), m_pigeon.getYaw( ), getPositions( ));

    {
      Pose2d botLLPose = RobotContainer.getInstance( ).m_vision.getLimelightValidPose(getPose( ));
      double latency = RobotContainer.getInstance( ).m_vision.getTargetLatency( );

      if (botLLPose != null && DriverStation.isTeleopEnabled( ))
      {
        //Adding a position specified by the limelight to the estimator at the time that the pose was generated 
        m_poseEstimator.addVisionMeasurement(botLLPose, Timer.getFPGATimestamp( ) - (latency / 1000));
      }

      Pose2d rawPose = RobotContainer.getInstance( ).m_vision.getLimelightRawPose( );

      if (rawPose != null && !m_isTeleported)
      {
        resetLimelightOdometry(rawPose);
        m_isTeleported = true;
      }
    }

    // Pose2d estimate = getPose();
    // DataLogManager.log(String.format("%$: X : %.3f Y %.3f", getSubsystem( ), estimate.getX( ), estimate.getY( )));
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Limelight driving mode
  //
  public void driveWithLimelightInit(boolean m_endAtTarget)
  {
    // get pid values from dashboard
    m_turnConstant = SmartDashboard.getNumber("DLL_turnConstant", m_turnConstant);
    m_turnPidKp = SmartDashboard.getNumber("DLL_turnPidKp", m_turnPidKp);
    m_turnPidKi = SmartDashboard.getNumber("DLL_turnPidKi", m_turnPidKi);
    m_turnPidKd = SmartDashboard.getNumber("DLL_turnPidKd", m_turnPidKd);
    m_turnMax = SmartDashboard.getNumber("DLL_turnMax", m_turnMax);

    m_throttlePidKp = SmartDashboard.getNumber("DLL_throttlePidKp", m_throttlePidKp);
    m_throttlePidKi = SmartDashboard.getNumber("DLL_throttlePidKi", m_throttlePidKi);
    m_throttlePidKd = SmartDashboard.getNumber("DLL_throttlePidKd", m_throttlePidKd);
    m_throttleMax = SmartDashboard.getNumber("DLL_throttleMax", m_throttleMax);
    m_throttleShape = SmartDashboard.getNumber("DLL_throttleShape", m_throttleShape);

    m_targetAngle = SmartDashboard.getNumber("DLL_targetAngle", m_targetAngle);
    m_setPointDistance = SmartDashboard.getNumber("DLL_setPointDistance", m_setPointDistance);
    m_angleThreshold = SmartDashboard.getNumber("DLL_angleThreshold", m_angleThreshold);
    m_distThreshold = SmartDashboard.getNumber("DLL_distThreshold", m_distThreshold);

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
      driveStop(false);
      if (m_limelightDebug >= 1)
        DataLogManager.log(getSubsystem( ) + ": DLL TV-FALSE - SIT STILL");

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
    SmartDashboard.putNumber("DLL_turnOutput", turnOutput);
    SmartDashboard.putNumber("DLL_throttleOutput", throttleOutput);
    SmartDashboard.putNumber("DLL_limeLightDist", m_limelightDistance);

    // cap max turn and throttle output
    turnOutput = MathUtil.clamp(turnOutput, -m_turnMax, m_turnMax);
    throttleOutput = MathUtil.clamp(throttleOutput, -m_throttleMax, m_throttleMax);

    // put turn and throttle outputs on the dashboard
    SmartDashboard.putNumber("DLL_turnClamped", turnOutput);
    SmartDashboard.putNumber("DLL_throttleClamped", throttleOutput);

    Translation2d llTranslation = new Translation2d(throttleOutput, 0);
    drive(llTranslation, turnOutput, false, true);

    if (m_limelightDebug >= 1)
      DataLogManager.log(String.format("%s: DLL tv: %d tx: %.2f ty: %.2f lldist: %.2f distErr: %.2f trnOut: %.2f thrOut: %2f",
      // @formatter:off
        getSubsystem(),
        tv,
        tx,
        ty,
        m_limelightDistance,
        Math.abs(m_setPointDistance - m_limelightDistance),
        turnOutput,
        throttleOutput
        // @formatter:on
      ));
  }

  public boolean driveWithLimelightIsFinished( )
  {
    RobotContainer rc = RobotContainer.getInstance( );
    boolean tv = rc.m_vision.getTargetValid( );
    double tx = rc.m_vision.getHorizOffsetDeg( );

    return (tv && ((Math.abs(tx)) <= m_angleThreshold)
        && (Math.abs(m_setPointDistance - m_limelightDistance) <= m_distThreshold));
  }

  public void driveWithLimelightEnd( )
  {
    driveStop(false);
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

    DataLogManager.log(String.format("%s: DLL tv: %d tx: %.2f ty: %.2f lldist: %.2f distErr: %.2f sanityCheck: %s",
    // @formatter:off
      getSubsystem(),
      tv,
      tx,
      ty,
      m_limelightDistance,
      Math.abs(m_setPointDistance - m_limelightDistance),
      ((sanityCheck) ? "PASSED" : "FAILED")
      // @formatter:on
    ));

    return sanityCheck;
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Autonomous mode - Holonomic path follower
  //
  public void driveWithPathFollowerInit(PathPlannerTrajectory trajectory, boolean useInitialPose)
  {
    m_holonomicController = new HolonomicDriveController(m_xController, m_yController, m_thetaController);

    m_trajectory = trajectory;

    m_field.getObject("trajectory").setTrajectory(m_trajectory);

    List<Trajectory.State> trajStates = new ArrayList<Trajectory.State>( );
    trajStates = m_trajectory.getStates( );
    DataLogManager.log(String.format("%s: PATH states: %d duration: %.3f secs", getSubsystem( ), trajStates.size( ),
        m_trajectory.getTotalTimeSeconds( )));

    // This initializes the odometry (where we are)
    if (useInitialPose)
    {
      resetOdometry(m_trajectory.getInitialHolonomicPose( ));
      DataLogManager.log("GYRO : " + m_pigeon.getYaw( ));
    }

    m_trajTimer.reset( );
    m_trajTimer.start( );
  }

  public void driveWithPathFollowerLimelightInit(PathPlannerTrajectory trajectory, boolean useInitialPose)
  {
    m_holonomicController = new HolonomicDriveController(m_xController, m_yController, m_thetaController);

    m_trajectory = trajectory;

    m_field.getObject("trajectory").setTrajectory(m_trajectory);

    List<Trajectory.State> trajStates = new ArrayList<Trajectory.State>( );
    trajStates = m_trajectory.getStates( );
    DataLogManager.log(String.format("%s: PATH states: %d duration: %.3f secs", getSubsystem( ), trajStates.size( ),
        m_trajectory.getTotalTimeSeconds( )));

    // This initializes the odometry (where we are)
    if (useInitialPose)
    {
      resetLimelightOdometry(m_trajectory.getInitialHolonomicPose( ));
    }

    m_trajTimer.reset( );
    m_trajTimer.start( );
  }

  public void driveWithPathFollowerExecute( )
  {
    Trajectory.State trajState = m_trajectory.sample(m_trajTimer.get( ));
    Pose2d currentPose = getPose( );

    ChassisSpeeds targetChassisSpeeds = m_holonomicController.calculate(currentPose, trajState,
        m_trajectory.getEndState( ).holonomicRotation/* trajState.poseMeters.getRotation( ) */); // TODO: find out what's wrong with getting desired rotation

    // Convert to module states
    SwerveModuleState[ ] moduleStates = SWConsts.swerveKinematics.toSwerveModuleStates(targetChassisSpeeds);

    double targetfrontLeft = (moduleStates[0].speedMetersPerSecond);
    double targetfrontRight = (moduleStates[1].speedMetersPerSecond);
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

    double targetHeading = m_trajectory.getEndState( ).holonomicRotation.getDegrees( );
    double currentHeading = currentPose.getRotation( ).getDegrees( );

    setModuleStates(moduleStates);

    if (m_pathDebug >= 1)
    {
      DataLogManager.log(String.format(
          "%s: PATH time: %.3f curXYR: %.2f %.2f %.2f targXYR %.2f %.2f %.1f chasXYO: %.1f %.1f %.1f targVel: %.1f %.1f %.1f %.1f curVel: %.2f %.2f %.2f %.2f errXYR: %.2f %.2f %.2f",
          // @formatter:off
          getSubsystem(),
          m_trajTimer.get( ),
          currentTrajX,
          currentTrajY,
          currentHeading,
          targetTrajX,
          targetTrajY,
          targetHeading,
          targetChassisSpeeds.vxMetersPerSecond,
          targetChassisSpeeds.vyMetersPerSecond,
          Units.radiansToDegrees(targetChassisSpeeds.omegaRadiansPerSecond),
          moduleStates[0].speedMetersPerSecond,
          moduleStates[1].speedMetersPerSecond,
          moduleStates[2].speedMetersPerSecond,
          moduleStates[3].speedMetersPerSecond,
          currentfrontLeft,
          currentfrontRight,
          currentbackLeft,
          currentbackRight,
          targetTrajX-currentTrajX,
          targetTrajY-currentTrajY,
          targetHeading-currentHeading
          // @formatter:on 
      ));
    }

    if (m_pathDebug >= 2)
    {
      // target velocity and its error
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_targetVelFrontLeft", targetfrontLeft);
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_targetVelFrontRight", targetfrontRight);
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_targetVelBackLeft", targetbackLeft);
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_targetVelBackRight", targetbackRight);
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_currentVelFrontLeft", currentfrontLeft);
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_currentVelFrontRight", currentfrontRight);
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_currentVelBackLeft", currentbackLeft);
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_currentVelBackRight", currentbackRight);

      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_velErrorFrontLeft", targetfrontLeft - currentfrontLeft);
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_velErrorFrontRight", targetfrontRight - currentfrontRight);
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_velErrorBackLeft", targetbackLeft - currentbackLeft);
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_velErrorBackRight", targetbackRight - currentbackRight);

      // target distance and its error
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_currentTrajX", targetTrajX);
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_currentTrajY", targetTrajY);
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_targetTrajX", currentTrajX);
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_targetTrajY", currentTrajY);

      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_trajErrorX", trajState.poseMeters.relativeTo(currentPose).getX( ));
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_trajErrorY", trajState.poseMeters.relativeTo(currentPose).getY( ));

      // target heading and its error
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_targetHeading", targetHeading);
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_currentHeading", currentHeading);
      SmartDashboard.putNumber(getSubsystem( ) + ": PATH_headingError",
          trajState.poseMeters.relativeTo(currentPose).getRotation( ).getDegrees( ));
    }
  }

  public boolean driveWithPathFollowerIsFinished( )
  {
    if (m_trajTimer.get( ) == 0)
      return false;

    if (m_trajTimer.get( ) >= 15.0)
    {
      DataLogManager.log(getSubsystem( ) + ": PATH - path follower timeout!");
      return true;
    }

    return (m_trajTimer.hasElapsed(m_trajectory.getTotalTimeSeconds( ) + 0.120));
  }

  public void driveWithPathFollowerEnd( )
  {
    m_trajTimer.stop( );
  }

  public void setIsTeleported(boolean isTeleported)
  {
    m_isTeleported = isTeleported;
  }

  //// 1678 Swerve //////////////////////////////////////////////////////////////

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
  {
    if (m_isSnapping)
    {
      if (Math.abs(rotation) <= Constants.kStickDeadband)
      {
        driveIsSnapFinished(false);
        rotation = driveSnapCalculate( );
      }
      else
      {
        driveIsSnapFinished(true);
      }
    }

    SwerveModuleState[ ] swerveModuleStates = null;
    if (m_locked)
    {
      swerveModuleStates = new SwerveModuleState[ ]
      {
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),  //
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)), //
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)), //
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
      };
    }
    else
    {
      swerveModuleStates = SWConsts.swerveKinematics.toSwerveModuleStates(fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX( ), translation.getY( ), rotation, m_pigeon.getYaw( ))
          : new ChassisSpeeds(translation.getX( ), translation.getY( ), rotation));
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SWConsts.maxSpeed);

    for (SwerveModule mod : m_swerveMods)
    {
      mod.setDesiredState(swerveModuleStates[mod.m_moduleNumber], isOpenLoop);
    }
  }

  public void driveStop(boolean fieldRelative)
  {
    drive(new Translation2d(0, 0), 0.0, fieldRelative, true);
  }

  public void driveBalanceExecute( )
  {
    double motorOutput;

    m_pitch = m_pigeon.getPitch( ).getDegrees( );
    if (Math.abs(m_pitch) > SWConsts.kDriveBalancedAngle)
    {
      motorOutput = SWConsts.kDriveBalanceKp * m_pitch;
      drive(new Translation2d(motorOutput, 0), 0, false, true);
    }
    else
    {
      driveStop(true);
    }
    //DataLogManager.log(String.format(getSubsystem() + ": Robot pitch: %.1f degrees - Robot power applied to motors: %.1f m/s", m_pitch, drivevalue));
  }

  //
  // Snap to a direction
  //  
  public void driveSnapInit(double snapAngle)
  {
    m_snapPIDController.reset(m_pigeon.getYaw( ).getRadians( ));
    m_snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
    m_isSnapping = true;
  }

  public double driveSnapCalculate( )
  {
    return m_snapPIDController.calculate(m_pigeon.getYaw( ).getRadians( ));
  }

  private boolean isSnapComplete( )
  {
    double error = m_snapPIDController.getGoal( ).position - m_pigeon.getYaw( ).getRadians( );
    return (Math.abs(error) < Math.toRadians(SnapConstants.kEpsilon));
  }

  public boolean driveIsSnapFinished(boolean force)
  {
    if (m_isSnapping && (force || isSnapComplete( )))
    {
      m_isSnapping = false;
      m_snapPIDController.reset(m_pigeon.getYaw( ).getRadians( ));
    }

    return !m_isSnapping;
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[ ] desiredStates)
  {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SWConsts.maxSpeed);

    for (SwerveModule mod : m_swerveMods)
    {
      mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
      SmartDashboard.putNumber(String.format("mod%d desired speed", mod.m_moduleNumber),
          desiredStates[mod.m_moduleNumber].speedMetersPerSecond);
      SmartDashboard.putNumber(String.format("mod%d desired angle", mod.m_moduleNumber),
          MathUtil.inputModulus(desiredStates[mod.m_moduleNumber].angle.getDegrees( ), 0, 180));
    }
  }

  //
  // Getters and setters
  //
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
    return m_poseEstimator.getEstimatedPosition( );
  }

  public void resetOdometry(Pose2d pose)
  {
    DataLogManager.log("Position Before : " + m_poseEstimator.getEstimatedPosition( ) + " Gyro : " + m_pigeon.getYaw( ));
    m_poseEstimator.resetPosition(m_pigeon.getYaw( ), getPositions( ), pose);
    DataLogManager.log("Position After : " + m_poseEstimator.getEstimatedPosition( ) + " Gyro : " + m_pigeon.getYaw( ));
  }

  public void resetLimelightOdometry(Pose2d pose)
  {
    m_poseEstimator.resetPosition(m_pigeon.getYaw( ), getPositions( ), pose);
  }

  public void resetOdometryToLimelight( )
  {
    Pose2d llPose = RobotContainer.getInstance( ).m_vision.getLimelightRawPose( );

    if (llPose != null)
    {
      resetLimelightOdometry(llPose);
    }
  }

  public void resetAnglesToAbsolute( )
  {
    for (SwerveModule mod : m_swerveMods)
    {
      mod.resetToAbsolute( );
    }
  }

  //
  // Used by WPILib in 2022 - switched to getPositions in 2023
  //
  // public SwerveModuleState[ ] getStates( )
  // {
  //   SwerveModuleState[ ] states = new SwerveModuleState[4];
  //   for (SwerveModule mod : m_swerveMods)
  //   {
  //     states[mod.m_moduleNumber] = mod.getState( );
  //     SmartDashboard.putNumber(String.format("mod%d current speed", mod.m_moduleNumber, "mod "),
  //         states[mod.m_moduleNumber].speedMetersPerSecond);
  //     SmartDashboard.putNumber(String.format("mod%d current angle", mod.m_moduleNumber, "mod "),
  //         MathUtil.inputModulus(states[mod.m_moduleNumber].angle.getDegrees( ), 0, 180));
  //   }
  //   return states;
  // }

  public SwerveModulePosition[ ] getPositions( )
  {
    SwerveModulePosition[ ] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : m_swerveMods)
    {
      positions[mod.m_moduleNumber] = mod.getPosition( );
      SmartDashboard.putNumber(String.format("mod%d current distance", mod.m_moduleNumber, "mod "),
          positions[mod.m_moduleNumber].distanceMeters);
      SmartDashboard.putNumber(String.format("mod%d current angle", mod.m_moduleNumber, "mod "),
          MathUtil.inputModulus(positions[mod.m_moduleNumber].angle.getDegrees( ), 0, 180));
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

  public double getYaw( )
  {
    return m_pigeon.getYaw( ).getDegrees( );
  }

  public void zeroGyro( )
  {
    zeroGyro(0.0);
  }

  public void zeroGyro(double reset)
  {
    m_pigeon.setYaw(reset);
  }

  //// 1678 Swerve //////////////////////////////////////////////////////////////
}
