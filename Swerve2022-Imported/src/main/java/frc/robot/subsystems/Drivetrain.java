
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DTConsts;
import frc.robot.Constants.LEDConsts.LEDColor;

/**
 *
 */
public class Drivetrain extends SubsystemBase
{
  private final WPI_Pigeon2           m_pigeonIMU          = new WPI_Pigeon2(0, DTConsts.kCANBusString);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter       m_xspeedLimiter      = new SlewRateLimiter(3);
  private final SlewRateLimiter       m_yspeedLimiter      = new SlewRateLimiter(3);
  private final SlewRateLimiter       m_rotLimiter         = new SlewRateLimiter(3);

  public static final double          kMaxSpeed            = 3.0;             // 3 meters per second
  public static final double          kMaxAngularSpeed     = Math.PI;         // 1/2 rotation per second

  private final Translation2d         m_frontLeftLocation  = new Translation2d(0.427, 0.427);
  private final Translation2d         m_frontRightLocation = new Translation2d(0.427, -0.427);
  private final Translation2d         m_backLeftLocation   = new Translation2d(-0.427, 0.427);
  private final Translation2d         m_backRightLocation  = new Translation2d(-0.427, -0.427);

  private final SwerveModule          m_frontLeft          =
      new SwerveModule(DTConsts.kLFDrive1CANID, DTConsts.kLFTurn2CANID, DTConsts.kLFCANCoderCANID);
  private final SwerveModule          m_frontRight         =
      new SwerveModule(DTConsts.kRFDrive3CANID, DTConsts.kRFTurn4CANID, DTConsts.kRFCANCoderCANID);
  private final SwerveModule          m_backLeft           =
      new SwerveModule(DTConsts.kLRDrive5CANID, DTConsts.kLRTurn6CANID, DTConsts.kLRCANCoderCANID);
  private final SwerveModule          m_backRight          =
      new SwerveModule(DTConsts.kRRDrive7CANID, DTConsts.kRRTurn8CANID, DTConsts.kRRCANCoderCANID);

  private final SwerveDriveKinematics m_kinematics         =
      new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry   m_odometry           = new SwerveDriveOdometry(m_kinematics, m_pigeonIMU.getRotation2d( ));

  // Limelight drive
  private double                      m_turnConstant       = DTConsts.kTurnConstant;
  private double                      m_turnPidKp          = DTConsts.kTurnPidKp;
  private double                      m_turnPidKi          = DTConsts.kTurnPidKi;
  private double                      m_turnPidKd          = DTConsts.kTurnPidKd;
  private double                      m_turnMax            = DTConsts.kTurnMax;
  private double                      m_throttlePidKp      = DTConsts.kThrottlePidKp;
  private double                      m_throttlePidKi      = DTConsts.kThrottlePidKi;
  private double                      m_throttlePidKd      = DTConsts.kThrottlePidKd;
  private double                      m_throttleMax        = DTConsts.kThrottleMax;
  private double                      m_throttleShape      = DTConsts.kThrottleShape;

  private double                      m_targetAngle        = DTConsts.kTargetAngle;      // Optimal shooting angle
  private double                      m_setPointDistance   = DTConsts.kSetPointDistance; // Optimal shooting distance
  private double                      m_angleThreshold     = DTConsts.kAngleThreshold;   // Tolerance around optimal
  private double                      m_distThreshold      = DTConsts.kDistThreshold;    // Tolerance around optimal

  // DriveWithLimelight pid controller objects
  private PIDController               m_turnPid            = new PIDController(0.0, 0.0, 0.0);
  private PIDController               m_throttlePid        = new PIDController(0.0, 0.0, 0.0);

  private int                         m_driveDebug         = 0;    // Debug flag to disable extra drive logging calls
  private int                         m_limelightDebug     = 0;    // Debug flag to disable extra limelight logging calls
  private int                         m_ramseteDebug       = 0;    // Debug flag to disable extra ramsete logging calls

  private boolean                     m_throttleZeroed     = false; // Throttle joystick zeroed safety check
  private boolean                     m_isQuickTurn        = false; // Quickturn mode active in curvature drive
  private boolean                     m_driveSlowMode      = false; // Slow drive mode active when climbing
  private double                      m_limelightDistance;
  private double                      m_offsetLeft;
  private double                      m_offsetRight;

  // Odometry and telemetry
  private double                      m_distanceLeft;              // Left wheel distance in meters
  private double                      m_distanceRight;             // Right wheel distance in meters

  /**
  *
  */
  public Drivetrain( )
  {
    setName("Drivetrain");
    setSubsystem("Drivetrain");
    //addChild("DiffDrive", m_diffDrive);    

    m_pigeonIMU.reset( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    updateOdometry( );
    updateDashboardValues( );
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

  private void initTalonMaster( )
  {

  }

  private void initTalonFollower(WPI_TalonFX motor, int master)
  {

  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Periodic helper methods
  //
  /** Updates the field relative position of the robot. */
  public void updateOdometry( )
  {
    m_odometry.update(m_pigeonIMU.getRotation2d( ), m_frontLeft.getState( ), m_frontRight.getState( ), m_backLeft.getState( ),
        m_backRight.getState( ));
  }

  private void updateDashboardValues( )
  {
    SmartDashboard.putNumber("SW_FLDrive", m_frontLeft.m_driveMotor.getMotorOutputPercent( ));
    SmartDashboard.putNumber("SW_FRDrive", m_frontRight.m_driveMotor.getMotorOutputPercent( ));
    SmartDashboard.putNumber("SW_BLDrive", m_backLeft.m_driveMotor.getMotorOutputPercent( ));
    SmartDashboard.putNumber("SW_BRDrive", m_backRight.m_driveMotor.getMotorOutputPercent( ));

    SmartDashboard.putNumber("SW_FLTurn", m_frontLeft.m_turningMotor.getMotorOutputPercent( ));
    SmartDashboard.putNumber("SW_FRTurn", m_frontRight.m_turningMotor.getMotorOutputPercent( ));
    SmartDashboard.putNumber("SW_BLTurn", m_backLeft.m_turningMotor.getMotorOutputPercent( ));
    SmartDashboard.putNumber("SW_BRTurn", m_backRight.m_turningMotor.getMotorOutputPercent( ));

    SmartDashboard.putNumber("SW_FLCANCoder", m_frontLeft.m_turningCANCoder.getPosition( ));
    SmartDashboard.putNumber("SW_FRCANCoder", m_frontRight.m_turningCANCoder.getPosition( ));
    SmartDashboard.putNumber("SW_BLCANCoder", m_backLeft.m_turningCANCoder.getPosition( ));
    SmartDashboard.putNumber("SW_BRCANCoder", m_backRight.m_turningCANCoder.getPosition( ));

    SmartDashboard.putNumber("SW_Heading", m_pigeonIMU.getAngle( ));
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Getters/Setters
  //
  // Wheel encoders
  //

  public void resetEncoders( )
  {

  }

  private int metersToNativeUnits(double meters)
  {
    return 0;
  }

  private double nativeUnitsToMeters(double nativeUnits)
  {
    return 0;
  }

  private int mpsToNativeUnits(double velocity)
  {
    return 0;
  }

  private double nativeUnitsToMPS(double nativeUnitsVelocity)
  {
    return 0;
  }

  private double getDistanceMetersLeft( )
  {

    return 0;

  }

  private double getDistanceMetersRight( )
  {
    return 0;
  }

  private double joystickOutputToNative(double output)
  {
    return 0;
  }

  private void velocityArcadeDrive( )
  {}

  //
  // Gyro
  //
  public void resetGyro( )
  {

  }

  private double getGyroHeading( )
  {
    return 0;
  }

  private void getYawPitchRoll( )
  {

  }

  //
  // Odometry
  //
  private void resetOdometry( )
  {

  }

  public Pose2d getPose( )
  {
    //TODO: if needed, change
    return m_odometry.getPoseMeters( );
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Set Talon brake/coast mode
  //
  private void setBrakeMode(boolean brakeMode)
  {

  }

  //
  // Voltage-based tank drive
  //
  public void tankDriveVolts(double left, double right)
  {}

  private void syncFollowerPIDFromDashboard( )
  {

  }

  //
  // Check if speed is below tolerance settings
  //
  public boolean driveIsStopped( )
  {
    return false;
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Trajectory management
  //
  private void plotTrajectory(Trajectory trajectory)
  {

  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////// Public Interfaces ///////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////
  //
  // Set quick turn for curvature drive
  //
  public void driveSetQuickTurn(boolean quickTurn)
  {
    m_isQuickTurn = quickTurn;
  }

  public void setDriveSlowMode(boolean driveSlowMode)
  {
    m_driveSlowMode = driveSlowMode;
  }

  public void driveStopMotors( )
  {

  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Teleop driving mode
  //

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed
   *          Speed of the robot in the x direction (forward).
   * @param ySpeed
   *          Speed of the robot in the y direction (sideways).
   * @param rot
   *          Angular rate of the robot.
   * @param fieldRelative
   *          Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative)
  {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_pigeonIMU.getRotation2d( ))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveWithJoystick(XboxController driverPad, boolean fieldRelative)
  {
    // Get x speed. Invert this because Xbox controllers return negative values when pushing forward.
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(driverPad.getLeftY( ), 0.02)) * Drivetrain.kMaxSpeed;

    // Get y speed or sideways/strafe speed. Invert this because a positive value is needed when
    // pulling left. Xbox controllers return positive values when pulling right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driverPad.getLeftX( ), 0.02)) * Drivetrain.kMaxSpeed;

    // Get rate of angular rotation. Invert this because a positive value is needed when pulling to
    // the left (CCW is positive in mathematics). Xbox controllers return positive values when pulling
    // to the right by default.
    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(driverPad.getRightX( ), 0.02)) * Drivetrain.kMaxAngularSpeed;

    drive(xSpeed, ySpeed, rot, fieldRelative);
  }

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

    // TODO: Add this to eliminate robot lurch
    // if (m_validL1)
    // m_driveL1.configOpenloopRamp(m_openLoopRamp);
    // if (m_validR3)
    // m_driveR3.configOpenloopRamp(m_openLoopRamp);

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
      //TODO: add back in
      //velocityArcadeDrive(0.0, 0.0);
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
    m_limelightDistance = rc.m_vision.getDistLimelight( );

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

    //TODO: add back in
    // if (m_validL1 || m_validR3)
    //   velocityArcadeDrive(throttleOutput, turnOutput);

    if (m_limelightDebug >= 1)
      DataLogManager.log(getSubsystem( )
      // @formatter:off
              + ": DTL tv: " + tv 
              + " tx: "      + String.format("%.1f", tx)
              + " ty: "      + String.format("%.1f", ty)
              + " lldist: "  + String.format("%.1f", m_limelightDistance)
              + " distErr: " + String.format("%.1f", Math.abs(m_setPointDistance - m_limelightDistance))
              //TODO: add back in
              //+ " stopped: " + driveIsStopped( )
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

    if (tv)
    {
      if (Math.abs(tx) <= m_angleThreshold)
        rc.m_led.setLLColor(LEDColor.LEDCOLOR_GREEN);
      else
      {
        if (tx < -m_angleThreshold)
          rc.m_led.setLLColor(LEDColor.LEDCOLOR_RED);
        else if (tx > m_angleThreshold)
          rc.m_led.setLLColor(LEDColor.LEDCOLOR_BLUE);
      }
    }
    else
      rc.m_led.setLLColor(LEDColor.LEDCOLOR_YELLOW);

    return (tv //
        && ((Math.abs(tx)) <= m_angleThreshold) //
        && (Math.abs(m_setPointDistance - m_limelightDistance) <= m_distThreshold)//
    //TODO: add back in
    //&& driveIsStopped( )
    );
  }

  public void driveWithLimelightEnd( )
  {
    //TODO: add back in
    // if (m_validL1 || m_validR3)
    //   velocityArcadeDrive(0.0, 0.0);

    // TODO: return settings back when command ends
    // if (m_validL1)
    // m_driveL1.configOpenloopRamp(0.0);
    // if (m_validR3)
    // m_driveR3.configOpenloopRamp(0.0);

    RobotContainer.getInstance( ).m_led.setLLColor(LEDColor.LEDCOLOR_OFF);
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

    DataLogManager.log(getSubsystem( )              //
        + ": DTL tv: " + tv                         //
        + " tx: " + tx                              //
        + " ty: " + ty                              //
        + " lldist: " + m_limelightDistance         //
        + " distErr: " + Math.abs(m_setPointDistance - m_limelightDistance) //
        + " sanityCheck: " + ((sanityCheck) ? "PASSED" : "FAILED")          //
    );

    return sanityCheck;
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Autonomous mode - Ramsete path follower
  //

  //TODO: finish for autos

}
