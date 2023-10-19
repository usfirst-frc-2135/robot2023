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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
      new SwerveDrivePoseEstimator(SWConsts.swerveKinematics, new Rotation2d( ), getPositions( ), new Pose2d( ));
  private Field2d                  m_field             = new Field2d( );

  // PID objects
  private ProfiledPIDController    m_snapPIDController =
      new ProfiledPIDController(SnapConstants.kP, SnapConstants.kI, SnapConstants.kD, SnapConstants.kThetaControllerConstraints);

  // Holonomic Drive Controller objects
  private HolonomicDriveController m_holonomicController;
  private PathPlannerTrajectory    m_trajectory;
  private Timer                    m_trajTimer         = new Timer( );
  private Pose2d                   m_poseBeforePath    = new Pose2d( );
  private Pose2d                   m_posePathStart     = new Pose2d( );
  private boolean                  m_isFieldRelative   = true;
  private boolean                  m_allowPoseEstimate = false;

  // Module variables
  private boolean                  m_isSnapping;
  private double                   m_snap_target;
  private Pose2d                   m_position          = new Pose2d( );
  private Rotation2d               m_heading           = new Rotation2d( );
  private double                   m_swerve_heading;
  private double                   m_robot_pitch;
  private double                   m_robot_roll;

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
  private PIDController            m_xController       = new PIDController(1, 0, 0);
  private PIDController            m_yController       = new PIDController(1, 0, 0);
  private ProfiledPIDController    m_thetaController   =
      new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  public Swerve( )
  {
    setName("Swerve");
    setSubsystem("Swerve");

    resetAnglesToAbsolute( );
    resetGyro(180.0); // All starting positions facing driver in field relative
    resetOdometry(new Pose2d(0, 0, m_heading));

    m_snapPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    initSmartDashboard( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    updateSwerveOdometry( );
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
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));
  }

  public void faultDump( )
  {
    DataLogManager.log(String.format("%s: faultDump  ----- DUMP FAULTS --------------", getSubsystem( )));
  }

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
  public void updateSwerveOdometry( )
  {
    m_heading = m_pigeon.getYaw( );

    m_poseEstimator.updateWithTime(Timer.getFPGATimestamp( ), m_heading, getPositions( ));

    if (m_allowPoseEstimate)
    {
      Vision vision = RobotContainer.getInstance( ).m_vision;
      Pose2d botLLPose = vision.getLimelightValidPose(getPose( ));
      double latency = vision.getTargetLatency( );

      //Adding a position specified by the limelight to the estimator at the time that the pose was generated 
      if (botLLPose != null && DriverStation.isTeleopEnabled( ))
        m_poseEstimator.addVisionMeasurement(botLLPose, Timer.getFPGATimestamp( ) - (latency / 1000));

      Pose2d rawPose = vision.getLimelightRawPose( );

      if (rawPose != null)
        resetOdometry(rawPose);
    }

    m_position = getPose( );

    // DataLogManager.log(String.format("%$: X : %.3f Y %.3f", getSubsystem( ), estimate.getX( ), estimate.getY( )));

    m_swerve_heading = MathUtil.inputModulus(m_heading.getDegrees( ), 0, 360);
    m_robot_pitch = m_pigeon.getPitch( ).getDegrees( );
    m_robot_roll = m_pigeon.getRoll( ).getDegrees( );

    m_snap_target = Math.toDegrees(m_snapPIDController.getGoal( ).position);
  }

  private void updateSmartDashboard( )
  {
    if (m_swerveDebug)
    {
      for (int i = 0; i < 4; i++)
      {
        SmartDashboard.putNumber("SWMod: %d - Speed", m_swerveMods[i].getState( ).speedMetersPerSecond);
        SmartDashboard.putNumber("SWMod: %d - Angle", m_swerveMods[i].getState( ).angle.getDegrees( ));
      }

      SmartDashboard.putNumber("SW: pose_x", m_position.getX( ));
      SmartDashboard.putNumber("SW: pose_y", m_position.getY( ));
      SmartDashboard.putNumber("SW: pose_rot", m_position.getRotation( ).getDegrees( ));

      SmartDashboard.putNumber("SW: swerve-hdg", m_swerve_heading);
      SmartDashboard.putNumber("SW: pitch", m_robot_pitch);
      SmartDashboard.putNumber("SW: roll", m_robot_roll);

      SmartDashboard.putNumber("SW: snap", m_snap_target);
    }

    m_field.setRobotPose(getPose( ));
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Limelight driving mode
  //
  public void driveWithLimelightInit(Vision vision, boolean m_endAtTarget)
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

    vision.m_tyfilter.reset( );
    vision.m_tvfilter.reset( );
    vision.syncStateFromDashboard( );
  }

  public void driveWithLimelightExecute(Vision vision)
  {
    boolean tv = vision.getTargetValid( );
    double tx = vision.getHorizOffsetDeg( );
    double ty = vision.getVertOffsetDeg( );

    if (!tv)
    {
      driveStop(false);
      if (m_limelightDebug >= 1)
        DataLogManager.log(String.format("%s: DLL TV-FALSE - SIT STILL", getSubsystem( )));

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
      DataLogManager.log(
          String.format("%s: DLL tv: %d tx: %.2f ty: %.2f lldist: %.2f distErr: %.2f trnOut: %.2f thrOut: %2f", getSubsystem( ),
              tv, tx, ty, m_limelightDistance, Math.abs(m_setPointDistance - m_limelightDistance), turnOutput, throttleOutput));
  }

  public boolean driveWithLimelightIsFinished(Vision vision)
  {
    boolean tv = vision.getTargetValid( );
    double tx = vision.getHorizOffsetDeg( );

    return (tv && ((Math.abs(tx)) <= m_angleThreshold)
        && (Math.abs(m_setPointDistance - m_limelightDistance) <= m_distThreshold));
  }

  public void driveWithLimelightEnd( )
  {
    driveStop(false);
  }

  public boolean isLimelightValid(Vision vision, double horizAngleRange, double distRange)
  {
    // check whether target is valid
    // check whether the limelight tx and ty is within a certain tolerance
    // check whether distance is within a certain tolerance
    boolean tv = vision.getTargetValid( );
    double tx = vision.getHorizOffsetDeg( );
    double ty = vision.getVertOffsetDeg( );
    m_limelightDistance = vision.getDistLimelight( );

    boolean sanityCheck =
        tv && (Math.abs(tx) <= horizAngleRange) && (Math.abs(m_setPointDistance - m_limelightDistance) <= distRange);
    // && (fabs(ty) <= vertAngleRange)

    DataLogManager.log(String.format("%s: DLL tv: %d tx: %.2f ty: %.2f lldist: %.2f distErr: %.2f check: %s", getSubsystem( ), tv,
        tx, ty, m_limelightDistance, Math.abs(m_setPointDistance - m_limelightDistance), ((sanityCheck) ? "PASSED" : "FAILED")));

    return sanityCheck;
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Autonomous mode - Holonomic path follower
  //
  public void driveWithPathFollowerInit(PathPlannerTrajectory trajectory, boolean useInitialPose)
  {
    m_trajectory = trajectory;

    m_holonomicController = new HolonomicDriveController(m_xController, m_yController, m_thetaController);

    m_field.getObject("trajectory").setTrajectory(m_trajectory);

    List<Trajectory.State> trajStates = new ArrayList<Trajectory.State>( );
    trajStates = m_trajectory.getStates( );
    DataLogManager.log(String.format("%s: PATH states: %d duration: %.3f secs", getSubsystem( ), trajStates.size( ),
        m_trajectory.getTotalTimeSeconds( )));

    // This initializes the odometry (where we are)
    if (useInitialPose)
    {
      m_poseBeforePath = getPose( );
      m_posePathStart = m_trajectory.getInitialHolonomicPose( );
      resetOdometry(m_posePathStart);
      m_isFieldRelative = false;
    }

    m_trajTimer.restart( );
  }

  public void driveWithPathFollowerExecute( )
  {
    Trajectory.State trajState = m_trajectory.sample(m_trajTimer.get( ));
    Pose2d currentPose = getPose( );

    ChassisSpeeds targetChassisSpeeds = m_holonomicController.calculate(currentPose, trajState,
        m_trajectory.getEndState( ).holonomicRotation/* trajState.poseMeters.getRotation( ) */); // TODO: find out what's wrong with getting desired rotation

    // Convert to module states
    SwerveModuleState[ ] moduleStates = SWConsts.swerveKinematics.toSwerveModuleStates(targetChassisSpeeds);

    double targetFL = (moduleStates[0].speedMetersPerSecond);
    double targetFR = (moduleStates[1].speedMetersPerSecond);
    double targetBL = (moduleStates[2].speedMetersPerSecond);
    double targetBR = (moduleStates[3].speedMetersPerSecond);

    double currentFL = m_swerveMods[0].getState( ).speedMetersPerSecond;
    double currentFR = m_swerveMods[1].getState( ).speedMetersPerSecond;
    double currentBL = m_swerveMods[2].getState( ).speedMetersPerSecond;
    double currentBR = m_swerveMods[3].getState( ).speedMetersPerSecond;

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
          getSubsystem( ), m_trajTimer.get( ), currentTrajX, currentTrajY, currentHeading, targetTrajX, targetTrajY,
          targetHeading, targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond,
          Units.radiansToDegrees(targetChassisSpeeds.omegaRadiansPerSecond), moduleStates[0].speedMetersPerSecond,
          moduleStates[1].speedMetersPerSecond, moduleStates[2].speedMetersPerSecond, moduleStates[3].speedMetersPerSecond,
          currentFL, currentFR, currentBL, currentBR, targetTrajX - currentTrajX, targetTrajY - currentTrajY,
          targetHeading - currentHeading));
    }

    if (m_pathDebug >= 2)
    {
      // target velocity and its error
      SmartDashboard.putNumber(String.format("%s: PATH_targetVelFrontLeft", getSubsystem( )), targetFL);
      SmartDashboard.putNumber(String.format("%s: PATH_targetVelFrontRight", getSubsystem( )), targetFR);
      SmartDashboard.putNumber(String.format("%s: PATH_targetVelBackLeft", getSubsystem( )), targetBL);
      SmartDashboard.putNumber(String.format("%s: PATH_targetVelBackRight", getSubsystem( )), targetBR);
      SmartDashboard.putNumber(String.format("%s: PATH_currentVelFrontLeft", getSubsystem( )), currentFL);
      SmartDashboard.putNumber(String.format("%s: PATH_currentVelFrontRight", getSubsystem( )), currentFR);
      SmartDashboard.putNumber(String.format("%s: PATH_currentVelBackLeft", getSubsystem( )), currentBL);
      SmartDashboard.putNumber(String.format("%s: PATH_currentVelBackRight", getSubsystem( )), currentBR);

      SmartDashboard.putNumber(String.format("%s: PATH_velErrorFrontLeft", getSubsystem( )), targetFL - currentFL);
      SmartDashboard.putNumber(String.format("%s: PATH_velErrorFrontRight", getSubsystem( )), targetFR - currentFR);
      SmartDashboard.putNumber(String.format("%s: PATH_velErrorBackLeft", getSubsystem( )), targetBL - currentBL);
      SmartDashboard.putNumber(String.format("%s: PATH_velErrorBackRight", getSubsystem( )), targetBR - currentBR);

      // target distance and its error
      SmartDashboard.putNumber(String.format("%s: PATH_currentTrajX", getSubsystem( )), targetTrajX);
      SmartDashboard.putNumber(String.format("%s: PATH_currentTrajY", getSubsystem( )), targetTrajY);
      SmartDashboard.putNumber(String.format("%s: PATH_targetTrajX", getSubsystem( )), currentTrajX);
      SmartDashboard.putNumber(String.format("%s: PATH_targetTrajY", getSubsystem( )), currentTrajY);

      SmartDashboard.putNumber(String.format("%s: PATH_trajErrorX", getSubsystem( )),
          trajState.poseMeters.relativeTo(currentPose).getX( ));
      SmartDashboard.putNumber(String.format("%s: PATH_trajErrorY", getSubsystem( )),
          trajState.poseMeters.relativeTo(currentPose).getY( ));

      // target heading and its error
      SmartDashboard.putNumber(String.format("%s: PATH_targetHeading", getSubsystem( )), targetHeading);
      SmartDashboard.putNumber(String.format("%s: PATH_currentHeading", getSubsystem( )), currentHeading);
      SmartDashboard.putNumber(String.format("%s: PATH_headingError", getSubsystem( )),
          trajState.poseMeters.relativeTo(currentPose).getRotation( ).getDegrees( ));
    }
  }

  public boolean driveWithPathFollowerIsFinished( )
  {
    if (m_trajTimer.get( ) >= Constants.kAutonomousPeriodSecs)
    {
      DataLogManager.log(String.format("%s: PATH - path follower timeout!", getSubsystem( )));
      return true;
    }

    return (m_trajTimer.hasElapsed(m_trajectory.getTotalTimeSeconds( ) + 0.120));
  }

  public void driveWithPathFollowerEnd( )
  {
    m_trajTimer.stop( );
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Helper functions
  //
  public Pose2d getPose( )
  {
    return m_poseEstimator.getEstimatedPosition( );
  }

  public void resetOdometry(Pose2d pose)
  {
    m_poseEstimator.resetPosition(m_heading, getPositions( ), pose);
    DataLogManager.log(String.format("%s: Reset position   : %s Gyro : %s", getSubsystem( ),
        m_poseEstimator.getEstimatedPosition( ).toString( ), m_heading.toString( )));
  }

  public void zeroGyro( )
  {
    resetGyro(0.0);
  }

  public void resetGyro(double resetAngle)
  {
    m_pigeon.setYaw(resetAngle);
    DataLogManager.log(String.format("%s: Gyro reset %.1f", getSubsystem( ), m_pigeon.getYaw( ).getDegrees( )));
  }

  public void enterTeleopMode( )
  {
    if (!m_isFieldRelative)
    {
      Pose2d currentPose = getPose( );

      resetOdometry(m_poseBeforePath.plus(currentPose.minus(m_posePathStart)));
      DataLogManager.log(String.format("%s: To Field Relative: %s", getSubsystem( ), getPose( ).toString( )));
      m_isFieldRelative = true;
    }
  }

  public void resetAnglesToAbsolute( )
  {
    for (SwerveModule mod : m_swerveMods)
      mod.resetToAbsolute( );
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
      mod.setDesiredState(swerveModuleStates[mod.m_moduleNumber], isOpenLoop);
  }

  public void driveBalanceExecute( )
  {
    double motorOutput;

    if (Math.abs(m_robot_pitch) > SWConsts.kDriveBalancedAngle)
    {
      motorOutput = SWConsts.kDriveBalanceKp * m_robot_pitch;
      drive(new Translation2d(motorOutput, 0), 0, false, true);
    }
    else
    {
      driveStop(true);
    }

    //DataLogManager.log(String.format(getSubsystem() + ": Robot pitch: %.1f degrees - Robot power applied to motors: %.1f m/s", m_robot_pitch, drivevalue));
  }

  public void driveStop(boolean fieldRelative)
  {
    drive(new Translation2d(0, 0), 0.0, fieldRelative, true);
  }

  //
  // Snap to a direction
  //  
  public void driveSnapInit(double snapAngle)
  {
    m_snapPIDController.reset(m_heading.getRadians( ));
    m_snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
    m_isSnapping = true;
  }

  public double driveSnapCalculate( )
  {
    return m_snapPIDController.calculate(m_heading.getRadians( ));
  }

  private boolean isSnapComplete( )
  {
    double error = m_snapPIDController.getGoal( ).position - m_heading.getRadians( );
    return (Math.abs(error) < Math.toRadians(SnapConstants.kEpsilon));
  }

  public boolean driveIsSnapFinished(boolean force)
  {
    if (m_isSnapping && (force || isSnapComplete( )))
    {
      m_isSnapping = false;
      m_snapPIDController.reset(m_heading.getRadians( ));
    }

    return !m_isSnapping;
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

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[ ] desiredStates)
  {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SWConsts.maxSpeed);

    for (SwerveModule mod : m_swerveMods)
      mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
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

  //// 1678 Swerve //////////////////////////////////////////////////////////////
}
