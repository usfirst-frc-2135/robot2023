
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.team1678.frc2022.drivers.Pigeon;
import frc.robot.team1678.frc2022.drivers.SwerveModule;
import frc.robot.team254.lib.util.TimeDelayedBoolean;

public class Swerve extends SubsystemBase
{
  public PeriodicIO            mPeriodicIO         = new PeriodicIO( );

  // status variable for being enabled
  public boolean               mIsEnabled          = false;

  // wants vision aim during auto
  public boolean               mWantsAutoVisionAim = false;

  public SwerveDriveOdometry   swerveOdometry;
  public SwerveModule[ ]       mSwerveMods;

  public Pigeon                mPigeon             = new Pigeon(Ports.kCANID_Pigeon2);

  // chassis velocity status
  ChassisSpeeds                chassisVelocity     = new ChassisSpeeds( );

  public boolean               isSnapping;
  private double               mLimelightVisionAlignGoal;
  private double               mGoalTrackVisionAlignGoal;
  private double               mVisionAlignAdjustment;

  public ProfiledPIDController snapPIDController;
  public PIDController         visionPIDController;

  // Private boolean to lock Swerve wheels
  private boolean              mLocked             = false;

  // Getter
  public boolean getLocked( )
  {
    return mLocked;
  }

  // Setter
  public void setLocked(boolean lock)
  {
    mLocked = lock;
  }

  public Swerve( )
  {
    setName("Swerve");
    setSubsystem("Swerve");

    swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, mPigeon.getYaw( ).getWPIRotation2d( ));

    snapPIDController = new ProfiledPIDController(Constants.SnapConstants.kP, Constants.SnapConstants.kI,
        Constants.SnapConstants.kD, Constants.SnapConstants.kThetaControllerConstraints);
    snapPIDController.enableContinuousInput(-Math.PI, Math.PI);

    visionPIDController = new PIDController(Constants.VisionAlignConstants.kP, Constants.VisionAlignConstants.kI,
        Constants.VisionAlignConstants.kD);
    visionPIDController.enableContinuousInput(-Math.PI, Math.PI);
    visionPIDController.setTolerance(0.0);

    zeroGyro( );

    mSwerveMods = new SwerveModule[ ]
    {
        new SwerveModule(0, Constants.SwerveConstants.Mod0.SwerveModuleConstants( )),
        new SwerveModule(1, Constants.SwerveConstants.Mod1.SwerveModuleConstants( )),
        new SwerveModule(2, Constants.SwerveConstants.Mod2.SwerveModuleConstants( )),
        new SwerveModule(3, Constants.SwerveConstants.Mod3.SwerveModuleConstants( ))
    };

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
    SmartDashboard.putNumber("SWMod: 1 - Speed", mSwerveMods[1].getState( ).speedMetersPerSecond);
    SmartDashboard.putNumber("SWMod: 1 - Angle", mSwerveMods[1].getState( ).angle.getDegrees( ));
    SmartDashboard.putNumber("SWMod: 2 - Speed", mSwerveMods[2].getState( ).speedMetersPerSecond);
    SmartDashboard.putNumber("SWMod: 2 - Angle", mSwerveMods[2].getState( ).angle.getDegrees( ));
    SmartDashboard.putNumber("SWMod: 3 - Speed", mSwerveMods[3].getState( ).speedMetersPerSecond);
    SmartDashboard.putNumber("SWMod: 3 - Angle", mSwerveMods[3].getState( ).angle.getDegrees( ));

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

    // m_turnConstant = SmartDashboard.getNumber("DTL_turnConstant",
    // m_turnConstant);
    // m_turnPidKp = SmartDashboard.getNumber("DTL_turnPidKp", m_turnPidKp);
    // m_turnPidKi = SmartDashboard.getNumber("DTL_turnPidKi", m_turnPidKi);
    // m_turnPidKd = SmartDashboard.getNumber("DTL_turnPidKd", m_turnPidKd);
    // m_turnMax = SmartDashboard.getNumber("DTL_turnMax", m_turnMax);

    // m_throttlePidKp = SmartDashboard.getNumber("DTL_throttlePidKp",
    // m_throttlePidKp);
    // m_throttlePidKi = SmartDashboard.getNumber("DTL_throttlePidKi",
    // m_throttlePidKi);
    // m_throttlePidKd = SmartDashboard.getNumber("DTL_throttlePidKd",
    // m_throttlePidKd);
    // m_throttleMax = SmartDashboard.getNumber("DTL_throttleMax", m_throttleMax);
    // m_throttleShape = SmartDashboard.getNumber("DTL_throttleShape",
    // m_throttleShape);

    // m_targetAngle = SmartDashboard.getNumber("DTL_targetAngle", m_targetAngle);
    // m_setPointDistance = SmartDashboard.getNumber("DTL_setPointDistance",
    // m_setPointDistance);
    // m_angleThreshold = SmartDashboard.getNumber("DTL_angleThreshold",
    // m_angleThreshold);
    // m_distThreshold = SmartDashboard.getNumber("DTL_distThreshold",
    // m_distThreshold);

    // // load in Pid constants to controller
    // m_turnPid = new PIDController(m_turnPidKp, m_turnPidKi, m_turnPidKd);
    // m_throttlePid = new PIDController(m_throttlePidKp, m_throttlePidKi,
    // m_throttlePidKd);

    // RobotContainer rc = RobotContainer.getInstance( );
    // rc.m_vision.m_tyfilter.reset( );
    // rc.m_vision.m_tvfilter.reset( );
    // rc.m_vision.syncStateFromDashboard( );
  }

  public void driveWithLimelightExecute( )
  {
    // RobotContainer rc = RobotContainer.getInstance( );
    // boolean tv = rc.m_vision.getTargetValid( );
    // double tx = rc.m_vision.getHorizOffsetDeg( );
    // double ty = rc.m_vision.getVertOffsetDeg( );

    // if (!tv)
    // {
    //   // TODO: add back in
    //   // velocityArcadeDrive(0.0, 0.0);
    //   if (m_limelightDebug >= 1)
    //     DataLogManager.log(getSubsystem( ) + ": DTL TV-FALSE - SIT STILL");
    //   return;
    // }

    // // get turn value - just horizontal offset from target
    // double turnOutput = -m_turnPid.calculate(tx, m_targetAngle);

    // if (turnOutput > 0)
    //   turnOutput = turnOutput + m_turnConstant;
    // else if (turnOutput < 0)
    //   turnOutput = turnOutput - m_turnConstant;

    // // get throttle value
    // m_limelightDistance = rc.m_vision.getDistLimelight( );

    // double throttleDistance = m_throttlePid.calculate(m_limelightDistance, m_setPointDistance);
    // double throttleOutput = throttleDistance * Math.pow(Math.cos(turnOutput * Math.PI / 180), m_throttleShape);

    // // put turn and throttle outputs on the dashboard
    // SmartDashboard.putNumber("DTL_turnOutput", turnOutput);
    // SmartDashboard.putNumber("DTL_throttleOutput", throttleOutput);
    // SmartDashboard.putNumber("DTL_limeLightDist", m_limelightDistance);

    // // cap max turn and throttle output
    // turnOutput = MathUtil.clamp(turnOutput, -m_turnMax, m_turnMax);
    // throttleOutput = MathUtil.clamp(throttleOutput, -m_throttleMax, m_throttleMax);

    // // put turn and throttle outputs on the dashboard
    // SmartDashboard.putNumber("DTL_turnClamped", turnOutput);
    // SmartDashboard.putNumber("DTL_throttleClamped", throttleOutput);

    // //TODO: add back in
    // // if (m_validL1 || m_validR3)
    // //   velocityArcadeDrive(throttleOutput, turnOutput);

    // if (m_limelightDebug >= 1)
    //   DataLogManager.log(getSubsystem( )
    //     // @formatter:off
    //     + ": DTL tv: " + tv 
    //     + " tx: "      + String.format("%.1f", tx)
    //     + " ty: "      + String.format("%.1f", ty)
    //     + " lldist: "  + String.format("%.1f", m_limelightDistance)
    //     + " distErr: " + String.format("%.1f", Math.abs(m_setPointDistance - m_limelightDistance))
    //     + " stopped: " + driveIsStopped( )
    //     + " trnOut: "  + String.format("%.2f", turnOutput)
    //     + " thrOut: "  + String.format("%.2f", throttleOutput)
    //     // @formatter:on
    //   );
  }

  public boolean driveWithLimelightIsFinished( )
  {
    // RobotContainer rc = RobotContainer.getInstance( );
    // boolean tv = rc.m_vision.getTargetValid( );
    // double tx = rc.m_vision.getHorizOffsetDeg( );

    // if (tv)
    // {
    //   if (Math.abs(tx) <= m_angleThreshold)
    //     rc.m_led.setLLColor(LEDColor.LEDCOLOR_GREEN);
    //   else
    //   {
    //     if (tx < -m_angleThreshold)
    //       rc.m_led.setLLColor(LEDColor.LEDCOLOR_RED);
    //     else if (tx > m_angleThreshold)
    //       rc.m_led.setLLColor(LEDColor.LEDCOLOR_BLUE);
    //   }
    // }
    // else
    //   rc.m_led.setLLColor(LEDColor.LEDCOLOR_YELLOW);

    // return (tv  && ((Math.abs(tx)) <= m_angleThreshold) 
    //    && (Math.abs(m_setPointDistance - m_limelightDistance) <= m_distThreshold)
    //    //TODO: add back in
    //    //&& driveIsStopped( )
    // );
    return true;
  }

  public void driveWithLimelightEnd( )
  {
    // TODO: add back in
    // if (m_validL1 || m_validR3)
    //   velocityArcadeDrive(0.0, 0.0);

    // RobotContainer.getInstance( ).m_led.setLLColor(LEDColor.LEDCOLOR_OFF);
  }

  public boolean isLimelightValid(double horizAngleRange, double distRange)
  {
    // // check whether target is valid
    // // check whether the limelight tx and ty is within a certain tolerance
    // // check whether distance is within a certain tolerance
    // RobotContainer rc = RobotContainer.getInstance( );
    // boolean tv = rc.m_vision.getTargetValid( );
    // double tx = rc.m_vision.getHorizOffsetDeg( );
    // double ty = rc.m_vision.getVertOffsetDeg( );
    // m_limelightDistance = rc.m_vision.getDistLimelight( );

    // boolean sanityCheck =
    // tv && (Math.abs(tx) <= horizAngleRange) 
    //    && (Math.abs(m_setPointDistance - m_limelightDistance) <= distRange);
    //    && (fabs(ty) <= vertAngleRange)

    // DataLogManager.log(getSubsystem( ) 
    //   + ": DTL tv: " + tv //
    //   + " tx: " + tx //
    //   + " ty: " + ty //
    //   + " lldist: " + m_limelightDistance //
    //   + " distErr: " + Math.abs(m_setPointDistance - m_limelightDistance) //
    //   + " sanityCheck: " + ((sanityCheck) ? "PASSED" : "FAILED") //
    // );

    // return sanityCheck;
    return true;
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Autonomous mode - Ramsete path follower
  //
  public void driveWithPathFollowerInit(Trajectory trajectory, boolean useInitialPose)
  {

  }

  public void driveWithPathFollowerExecute( )
  {

  }

  public boolean driveWithPathFollowerIsFinished( )
  {
    return true;
  }

  public void driveWithPathFollowerEnd( )
  {

  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Autonomous mode - Ramsete path follower
  //

  // TODO: finish for autos

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
    double angleAdjustment = snapPIDController.calculate(mPigeon.getYaw( ).getRadians( ));
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
    if (mLocked)
    {
      swerveModuleStates = new SwerveModuleState[ ]
      {
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)), new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)), new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
      };
    }
    else
    {
      swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX( ), translation.getY( ), rotation,
                  mPigeon.getYaw( ).getWPIRotation2d( ))
              : new ChassisSpeeds(translation.getX( ), translation.getY( ), rotation));
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods)
    {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void acceptLatestGoalTrackVisionAlignGoal(double vision_goal)
  {
    mGoalTrackVisionAlignGoal = vision_goal;
  }

  public void chooseVisionAlignGoal( )
  {
    double currentAngle = mPigeon.getYaw( ).getRadians( );

    mVisionAlignAdjustment = visionPIDController.calculate(currentAngle);
  }

  public double calculateSnapValue( )
  {
    return snapPIDController.calculate(mPigeon.getYaw( ).getRadians( ));
  }

  public void startSnap(double snapAngle)
  {
    snapPIDController.reset(mPigeon.getYaw( ).getRadians( ));
    snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
    isSnapping = true;
  }

  TimeDelayedBoolean delayedBoolean = new TimeDelayedBoolean( );

  private boolean snapComplete( )
  {
    double error = snapPIDController.getGoal( ).position - mPigeon.getYaw( ).getRadians( );
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
      snapPIDController.reset(mPigeon.getYaw( ).getRadians( ));
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
    swerveOdometry.resetPosition(pose, pose.getRotation( ));
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

  // @Override
  public void zeroSensors( )
  {
    zeroGyro(0.0);
  }

  public void zeroGyro( )
  {
    zeroGyro(0.0);
  }

  public void zeroGyro(double reset)
  {
    mPigeon.setYaw(reset);
    visionPIDController.reset( );
  }

  public void updateSwerveOdometry( )
  {
    swerveOdometry.update(mPigeon.getYaw( ).getWPIRotation2d( ), getStates( ));

    chassisVelocity = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(mSwerveMods[0].getState( ),
        mSwerveMods[1].getState( ), mSwerveMods[2].getState( ), mSwerveMods[3].getState( ));
  }

  // @Override
  public void stop( )
  {
    mIsEnabled = false;
  }

  // @Override
  public boolean checkSystem( )
  {
    return true;
  }

  // @Override
  public void readPeriodicInputs( )
  {
    mPeriodicIO.odometry_pose_x = swerveOdometry.getPoseMeters( ).getX( );
    mPeriodicIO.odometry_pose_y = swerveOdometry.getPoseMeters( ).getY( );
    mPeriodicIO.odometry_pose_rot = swerveOdometry.getPoseMeters( ).getRotation( ).getDegrees( );
    mPeriodicIO.pigeon_heading = mPigeon.getYaw( ).getDegrees( );
    mPeriodicIO.robot_pitch = mPigeon.getUnadjustedPitch( ).getDegrees( );
    mPeriodicIO.robot_roll = mPigeon.getRoll( ).getDegrees( );
    mPeriodicIO.snap_target = Math.toDegrees(snapPIDController.getGoal( ).position);
    mPeriodicIO.vision_align_target_angle = Math.toDegrees(mLimelightVisionAlignGoal);
    mPeriodicIO.swerve_heading = MathUtil.inputModulus(mPigeon.getYaw( ).getDegrees( ), 0, 360);

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

    public double angular_velocity;
    public double goal_velocity;

    public double profile_position;

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
  //   headers.add("snap_target");
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
  //   items.add(mIsEnabled ? 1.0 : 0.0);
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

  //   // send data to logging storage
  //   mStorage.addData(items);
  // }

}
