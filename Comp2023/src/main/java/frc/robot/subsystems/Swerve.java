
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SWConsts;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.team1678.frc2022.drivers.Pigeon;
import frc.robot.team1678.frc2022.drivers.SwerveModule;
import frc.robot.team254.lib.util.TimeDelayedBoolean;

public class Swerve extends SubsystemBase
{
  public PeriodicIO                 mPeriodicIO         = new PeriodicIO( );

  // wants vision aim during auto
  public boolean                    mWantsAutoVisionAim = false;

  public SwerveModulePosition[ ]    swervePositions;
  public SwerveDriveOdometry        swerveOdometry;
  public SwerveModule[ ]            mSwerveMods;

  //Odometery and telemetry
  public Pigeon                     m_pigeon            = new Pigeon(Ports.kCANID_Pigeon2);
  private Field2d                   m_field             = new Field2d( );

  // chassis velocity status
  public ChassisSpeeds              chassisVelocity     = new ChassisSpeeds( );

  public boolean                    isSnapping;
  private double                    mLimelightVisionAlignGoal;
  private double                    mVisionAlignAdjustment;

  public ProfiledPIDController      snapPIDController   = new ProfiledPIDController(Constants.SnapConstants.kP,
      Constants.SnapConstants.kI, Constants.SnapConstants.kD, Constants.SnapConstants.kThetaControllerConstraints);
  public PIDController              visionPIDController =
      new PIDController(Constants.VisionAlignConstants.kP, Constants.VisionAlignConstants.kI, Constants.VisionAlignConstants.kD);

  // Private boolean to lock Swerve wheels
  private boolean                   m_locked            = false;

  // Holonomic Drive Controller objects
  private HolonomicDriveController  m_holonomicController;
  private Trajectory                m_trajectory;
  private Timer                     m_trajTimer         = new Timer( );

  // Limelight drive
  private double                    m_turnConstant      = SWConsts.kTurnConstant;
  private double                    m_turnPidKp         = SWConsts.kTurnPidKp;
  private double                    m_turnPidKi         = SWConsts.kTurnPidKi;
  private double                    m_turnPidKd         = SWConsts.kTurnPidKd;
  private double                    m_turnMax           = SWConsts.kTurnMax;
  private double                    m_throttlePidKp     = SWConsts.kThrottlePidKp;
  private double                    m_throttlePidKi     = SWConsts.kThrottlePidKi;
  private double                    m_throttlePidKd     = SWConsts.kThrottlePidKd;
  private double                    m_throttleMax       = SWConsts.kThrottleMax;
  private double                    m_throttleShape     = SWConsts.kThrottleShape;

  private double                    m_targetAngle       = SWConsts.kTargetAngle; // Optimal shooting angle
  private double                    m_setPointDistance  = SWConsts.kSetPointDistance; // Optimal shooting distance
  private double                    m_angleThreshold    = SWConsts.kAngleThreshold; // Tolerance around optimal
  private double                    m_distThreshold     = SWConsts.kDistThreshold;// Tolerance around optimal

  private static final List<Pose3d> m_targetPoses       =
      Collections.unmodifiableList(List.of(new Pose3d(new Translation3d(7.24310, -2.93659, 0), new Rotation3d(0, 0, 0)), // AprilTag ID: 1 
          new Pose3d(new Translation3d(7.24310, -1.26019, 0), new Rotation3d(0, 0, 0)), // AprilTag ID: 2 
          new Pose3d(new Translation3d(7.24310, 0.41621, 0), new Rotation3d(0, 0, 0)), // AprilTag ID: 3 
          new Pose3d(new Translation3d(7.24310, 2.74161, 0), new Rotation3d(0, 0, 0)), // AprilTag ID: 4 
          new Pose3d(new Translation3d(-7.24310, 2.74161, 0), new Rotation3d(0, 0, 0)), // AprilTag ID: 5 
          new Pose3d(new Translation3d(-7.24310, 0.46272, 0), new Rotation3d(0, 0, 0)), // AprilTag ID: 6 
          new Pose3d(new Translation3d(-7.24310, -1.26019, 0), new Rotation3d(0, 0, 0)), // AprilTag ID: 7
          new Pose3d(new Translation3d(-7.24310, -2.74161, 0), new Rotation3d(0, 0, 0)) // AprilTag ID: 8
      ));

  // DriveWithLimelight pid controller objects
  private int                       m_limelightDebug    = 0; // Debug flag to disable extra limelight logging calls
  private PIDController             m_turnPid           = new PIDController(0.0, 0.0, 0.0);
  private PIDController             m_throttlePid       = new PIDController(0.0, 0.0, 0.0);
  private double                    m_limelightDistance;

  // Getter
  public boolean getLocked( )
  {
    return m_locked;
  }

  // Setter
  public void setLocked(boolean lock)
  {
    m_locked = lock;
  }

  public Swerve( )
  {
    setName("Swerve");
    setSubsystem("Swerve");

    zeroGyro( );

    mSwerveMods = new SwerveModule[ ]
    {
        new SwerveModule(0, Constants.SwerveConstants.Mod0.SwerveModuleConstants( )),
        new SwerveModule(1, Constants.SwerveConstants.Mod1.SwerveModuleConstants( )),
        new SwerveModule(2, Constants.SwerveConstants.Mod2.SwerveModuleConstants( )),
        new SwerveModule(3, Constants.SwerveConstants.Mod3.SwerveModuleConstants( ))
    };

    swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, m_pigeon.getYaw( ).getWPIRotation2d( ),
        new SwerveModulePosition[ ]
        {
            mSwerveMods[0].getPosition( ), mSwerveMods[1].getPosition( ), mSwerveMods[2].getPosition( ),
            mSwerveMods[3].getPosition( )
        });

    snapPIDController.enableContinuousInput(-Math.PI, Math.PI);

    visionPIDController.enableContinuousInput(-Math.PI, Math.PI);
    visionPIDController.setTolerance(0.0);

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
  {

  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Periodic helper methods
  //
  private void updateSmartDashboard( )
  {
    SmartDashboard.putNumber("SWMod: 0 - Speed", mSwerveMods[0].getState( ).speedMetersPerSecond);
    SmartDashboard.putNumber("SWMod: 0 - Angle", mSwerveMods[0].getState( ).angle.getDegrees( ));
    SmartDashboard.putNumber("SWMod: 0 - Dist", mSwerveMods[0].getPosition( ).distanceMeters);
    SmartDashboard.putNumber("SWMod: 1 - Speed", mSwerveMods[1].getState( ).speedMetersPerSecond);
    SmartDashboard.putNumber("SWMod: 1 - Angle", mSwerveMods[1].getState( ).angle.getDegrees( ));
    SmartDashboard.putNumber("SWMod: 1 - Dist", mSwerveMods[1].getPosition( ).distanceMeters);
    SmartDashboard.putNumber("SWMod: 2 - Speed", mSwerveMods[2].getState( ).speedMetersPerSecond);
    SmartDashboard.putNumber("SWMod: 2 - Angle", mSwerveMods[2].getState( ).angle.getDegrees( ));
    SmartDashboard.putNumber("SWMod: 2 - Dist", mSwerveMods[2].getPosition( ).distanceMeters);
    SmartDashboard.putNumber("SWMod: 3 - Speed", mSwerveMods[3].getState( ).speedMetersPerSecond);
    SmartDashboard.putNumber("SWMod: 3 - Angle", mSwerveMods[3].getState( ).angle.getDegrees( ));
    SmartDashboard.putNumber("SWMod: 3 - Dist", mSwerveMods[3].getPosition( ).distanceMeters);

    SmartDashboard.putNumber("SW: pose_x", mPeriodicIO.odometry_pose_x);
    SmartDashboard.putNumber("SW: pose_y", mPeriodicIO.odometry_pose_y);
    SmartDashboard.putNumber("SW: pose_rot", mPeriodicIO.odometry_pose_rot);

    SmartDashboard.putNumber("SW: pigeon-hdg", mPeriodicIO.pigeon_heading);
    SmartDashboard.putNumber("SW: pitch", mPeriodicIO.robot_pitch);
    SmartDashboard.putNumber("SW: roll", mPeriodicIO.robot_roll);

    SmartDashboard.putNumber("SW: snap", mPeriodicIO.snap_target);
    SmartDashboard.putNumber("SW: vision", mPeriodicIO.vision_align_target_angle);
    SmartDashboard.putNumber("SW: swerve-hdg", mPeriodicIO.swerve_heading);
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

  private void plotTrajectory(Trajectory trajectory)
  {
    m_field.getObject("trajectory").setTrajectory(trajectory);
  }

  public void driveWithPathFollowerInit(Trajectory trajectory, boolean useInitialPose)
  {
    m_holonomicController = new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0),
        new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)));

    m_trajectory = trajectory;

    if (!RobotBase.isReal( ))
    {
      plotTrajectory(m_trajectory);
    }

    List<Trajectory.State> trajStates = new ArrayList<Trajectory.State>( );
    trajStates = m_trajectory.getStates( );
    DataLogManager.log(getSubsystem( ) + ": DTR states: " + trajStates.size( ) + " dur: " + m_trajectory.getTotalTimeSeconds( ));

    // This initializes the odometry (where we are)
    if (useInitialPose)
      resetOdometry(m_trajectory.getInitialPose( ));

    m_trajTimer.reset( );
    m_trajTimer.start( );

    m_field.setRobotPose(getPose( ));
  }

  public void driveWithPathFollowerExecute( )
  {
    Trajectory.State trajState = m_trajectory.sample(m_trajTimer.get( ));
    Pose2d currentPose = getPose( );

    //TODO: how to find the goal position

    // Get the adjusted speeds. Here, we want the robot to be facing
    // 70 degrees (in the field-relative coordinate system).
    //TODO: update the degrees to desired coordinate system
    ChassisSpeeds targetChassisSpeeds = m_holonomicController.calculate(currentPose, trajState, Rotation2d.fromDegrees(70.0));
    // Convert to module states
    SwerveModuleState[ ] moduleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(targetChassisSpeeds);

    // Front left module state
    SwerveModuleState frontLeft = moduleStates[0];

    // Front right module state
    SwerveModuleState frontRight = moduleStates[1];

    // Back left module state
    SwerveModuleState backLeft = moduleStates[2];

    // Back right module state
    SwerveModuleState backRight = moduleStates[3];

    double targetfrontLeft = (frontLeft.speedMetersPerSecond); //TODO: Add MPS ??
    double targetfrontRight = (frontRight.speedMetersPerSecond); //TODO: Add MPS ??
    double targetbackLeft = (backLeft.speedMetersPerSecond);
    double targetbackRight = (backRight.speedMetersPerSecond);

    double currentfrontLeft = mSwerveMods[0].getState( ).speedMetersPerSecond;
    double currentfrontRight = mSwerveMods[1].getState( ).speedMetersPerSecond;
    double currentbackLeft = mSwerveMods[2].getState( ).speedMetersPerSecond;
    double currentbackRight = mSwerveMods[3].getState( ).speedMetersPerSecond;

    double targetTrajX = trajState.poseMeters.getX( );
    double targetTrajY = trajState.poseMeters.getY( );
    double currentTrajX = currentPose.getX( );
    double currentTrajY = currentPose.getY( );

    double targetHeading = trajState.poseMeters.getRotation( ).getDegrees( ); ///Maybe get in radians?
    double currentHeading = currentPose.getRotation( ).getDegrees( ); ///Maybe get in radians?

  }

  public boolean driveWithPathFollowerIsFinished( )
  {
    return true;
  }

  public void driveWithPathFollowerEnd( )
  {

  }

  //// 1678 Swerve //////////////////////////////////////////////////////////////

  public void setWantAutoVisionAim(boolean aim)
  {
    mWantsAutoVisionAim = aim;
  }

  public boolean getWantAutoVisionAim( )
  {
    return mWantsAutoVisionAim;
  }

  public void visionAlignDrive(Translation2d translation2d, boolean fieldRelative)
  {
    drive(translation2d, mVisionAlignAdjustment, fieldRelative, false);
  }

  public void angleAlignDrive(Translation2d translation2d, double targetHeading, boolean fieldRelative)
  {
    snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(targetHeading), 0.0));
    double angleAdjustment = snapPIDController.calculate(m_pigeon.getYaw( ).getRadians( ));
    drive(translation2d, angleAdjustment, fieldRelative, false);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
  {
    if (isSnapping)
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
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)), new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)), new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
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

    for (SwerveModule mod : mSwerveMods)
    {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void chooseVisionAlignGoal( )
  {
    double currentAngle = m_pigeon.getYaw( ).getRadians( );

    mVisionAlignAdjustment = visionPIDController.calculate(currentAngle);
  }

  public double calculateSnapValue( )
  {
    return snapPIDController.calculate(m_pigeon.getYaw( ).getRadians( ));
  }

  public void startSnap(double snapAngle)
  {
    snapPIDController.reset(m_pigeon.getYaw( ).getRadians( ));
    snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
    isSnapping = true;
  }

  TimeDelayedBoolean delayedBoolean = new TimeDelayedBoolean( );

  private boolean snapComplete( )
  {
    double error = snapPIDController.getGoal( ).position - m_pigeon.getYaw( ).getRadians( );
    return delayedBoolean.update(Math.abs(error) < Math.toRadians(Constants.SnapConstants.kEpsilon),
        Constants.SnapConstants.kTimeout);
  }

  public void maybeStopSnap(boolean force)
  {
    if (!isSnapping)
    {
      return;
    }
    if (force || snapComplete( ))
    {
      isSnapping = false;
      snapPIDController.reset(m_pigeon.getYaw( ).getRadians( ));
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[ ] desiredStates)
  {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods)
    {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
      SmartDashboard.putNumber("mod " + mod.moduleNumber + " desired speed",
          desiredStates[mod.moduleNumber].speedMetersPerSecond);
      SmartDashboard.putNumber("mod " + mod.moduleNumber + " desired angle",
          MathUtil.inputModulus(desiredStates[mod.moduleNumber].angle.getDegrees( ), 0, 180));
    }
  }

  public Pose2d getPose( )
  {
    return swerveOdometry.getPoseMeters( );
  }

  public void resetOdometry(Pose2d pose)
  {
    swerveOdometry.resetPosition(pose.getRotation( ), getPosition( ), pose);
    zeroGyro(pose.getRotation( ).getDegrees( ));
  }

  public void resetAnglesToAbsolute( )
  {
    for (SwerveModule mod : mSwerveMods)
    {
      mod.resetToAbsolute( );
    }
  }

  public SwerveModuleState[ ] getStates( )
  {
    SwerveModuleState[ ] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods)
    {
      states[mod.moduleNumber] = mod.getState( );
      SmartDashboard.putNumber("mod " + mod.moduleNumber + " current speed", states[mod.moduleNumber].speedMetersPerSecond);
      SmartDashboard.putNumber("mod " + mod.moduleNumber + " current angle",
          MathUtil.inputModulus(states[mod.moduleNumber].angle.getDegrees( ), 0, 180));
    }
    return states;
  }

  public SwerveModulePosition[ ] getPosition( )
  {
    SwerveModulePosition[ ] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods)
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
    for (SwerveModule swerveModule : mSwerveMods)
    {
      swerveModule.updateAnglePID(kP, kI, kD);
    }
  }

  public double[ ] getAnglePIDValues(int index)
  {
    return mSwerveMods[index].getAnglePIDValues( );
  }

  public void setVisionAlignPIDValues(double kP, double kI, double kD)
  {
    visionPIDController.setPID(kP, kI, kD);
  }

  public double[ ] getVisionAlignPIDValues( )
  {
    return new double[ ]
    {
        visionPIDController.getP( ), visionPIDController.getI( ), visionPIDController.getD( )
    };
  }

  public void zeroGyro( )
  {
    zeroGyro(0.0);
  }

  public void zeroGyro(double reset)
  {
    m_pigeon.setYaw(reset);
    visionPIDController.reset( );
  }

  public void updateSwerveOdometry( )
  {
    swerveOdometry.update(m_pigeon.getYaw( ).getWPIRotation2d( ), getPosition( ));

    chassisVelocity = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(mSwerveMods[0].getState( ),
        mSwerveMods[1].getState( ), mSwerveMods[2].getState( ), mSwerveMods[3].getState( ));
  }

  // @Override
  public void readPeriodicInputs( )
  {
    mPeriodicIO.odometry_pose_x = swerveOdometry.getPoseMeters( ).getX( );
    mPeriodicIO.odometry_pose_y = swerveOdometry.getPoseMeters( ).getY( );
    mPeriodicIO.odometry_pose_rot = swerveOdometry.getPoseMeters( ).getRotation( ).getDegrees( );
    mPeriodicIO.pigeon_heading = m_pigeon.getYaw( ).getDegrees( );
    mPeriodicIO.robot_pitch = m_pigeon.getUnadjustedPitch( ).getDegrees( );
    mPeriodicIO.robot_roll = m_pigeon.getRoll( ).getDegrees( );
    mPeriodicIO.snap_target = Math.toDegrees(snapPIDController.getGoal( ).position);
    mPeriodicIO.vision_align_target_angle = Math.toDegrees(mLimelightVisionAlignGoal);
    mPeriodicIO.swerve_heading = MathUtil.inputModulus(m_pigeon.getYaw( ).getDegrees( ), 0, 360);

    // SendLog( );
  }

  public static class PeriodicIO
  {
    // inputs
    public double odometry_pose_x;
    public double odometry_pose_y;
    public double odometry_pose_rot;

    public double pigeon_heading;
    public double robot_pitch;
    public double robot_roll;
    public double vision_align_target_angle;
    public double swerve_heading;

    // outputs
    public double snap_target;
  }

  // //logger
  // // @Override
  // public void registerLogger(LoggingSystem LS)
  // {
  //   SetupLog( );
  //   LS.register(mStorage, "SWERVE_LOGS.csv");
  // }

  // public void SetupLog( )
  // {
  //   mStorage = new LogStorage<PeriodicIO>( );

  //   ArrayList<String> headers = new ArrayList<String>( );
  //   headers.add("timestamp");
  //   headers.add("is_enabled");
  //   headers.add("odometry_pose_x");
  //   headers.add("odometry_pose_y");
  //   headers.add("odometry_pose_rot");
  //   headers.add("pigeon_heading");
  //   headers.add("robot_pitch");
  //   headers.add("robot_roll");
  //   headers.add("vision_align_target_angle");
  //   headers.add("swerve_heading");
  //   for (SwerveModule module : this.mSwerveMods)
  //   {
  //     headers.add(module.moduleNumber + "_angle");
  //     headers.add(module.moduleNumber + "_desired_angle");
  //     headers.add(module.moduleNumber + "_velocity");
  //     headers.add(module.moduleNumber + "_cancoder");
  //   }

  //   mStorage.setHeaders(headers);
  // }

  // public void SendLog( )
  // {
  //   ArrayList<Number> items = new ArrayList<Number>( );
  //   items.add(Timer.getFPGATimestamp( ));
  //   items.add(mPeriodicIO.odometry_pose_x);
  //   items.add(mPeriodicIO.odometry_pose_y);
  //   items.add(mPeriodicIO.odometry_pose_rot);
  //   items.add(mPeriodicIO.pigeon_heading);
  //   items.add(mPeriodicIO.robot_pitch);
  //   items.add(mPeriodicIO.robot_roll);
  //   items.add(mPeriodicIO.snap_target);
  //   items.add(mPeriodicIO.vision_align_target_angle);
  //   items.add(mPeriodicIO.swerve_heading);
  //   for (SwerveModule module : this.mSwerveMods)
  //   {
  //     items.add(module.getState( ).angle.getDegrees( ));
  //     items.add(module.getTargetAngle( ));
  //     items.add(module.getState( ).speedMetersPerSecond);
  //     items.add(MathUtil.inputModulus(module.getCanCoder( ).getDegrees( ) - module.angleOffset, 0, 360));
  //   }

  //   // // send data to logging storage
  //   mStorage.addData(items);
  // }

  //// 1678 Swerve //////////////////////////////////////////////////////////////

}
