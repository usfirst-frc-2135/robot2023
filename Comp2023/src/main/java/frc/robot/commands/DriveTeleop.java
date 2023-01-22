
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class DriveTeleop extends CommandBase
{
  private final Swerve          m_swerve;
  private final XboxController  m_driverPad;

  private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter    = new SlewRateLimiter(3);

  public DriveTeleop(Swerve swerve, XboxController driverPad)
  {
    m_swerve = swerve;
    m_driverPad = driverPad;

    setName("DriveTeleop");
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    driveWithGamepad(m_swerve, m_driverPad, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return false;
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param driverPad
   *          XboxController used by driver.
   * @param fieldRelative
   *          Whether the provided x and y speeds are relative to the field.
   */
  public void driveWithGamepad(Swerve swerve, XboxController driverPad, boolean fieldRelative)
  {
    // Get x speed. Invert this because Xbox controllers return negative values when pushing forward.
    final var xSpeed = m_xSpeedLimiter.calculate(MathUtil.applyDeadband(driverPad.getLeftY( ), Constants.kStickDeadband))

        * SwerveConstants.maxSpeed;

    // Get y speed or sideways/strafe speed. Invert this because a positive value is needed when
    // pulling left. Xbox controllers return positive values when pulling right by default.
    final var ySpeed = m_ySpeedLimiter.calculate(MathUtil.applyDeadband(driverPad.getLeftX( ), Constants.kStickDeadband))

        * SwerveConstants.maxSpeed;

    // Get rate of angular rotation. Invert this because a positive value is needed when pulling to
    // the left (CCW is positive in mathematics). Xbox controllers return positive values when pulling
    // to the right by default.
    final var rot =
        m_rotLimiter.calculate(MathUtil.applyDeadband(driverPad.getRightX( ), 0.02)) * SwerveConstants.maxAngularVelocity;

    Translation2d swerveTranslation = new Translation2d(xSpeed, ySpeed);

    swerve.drive(swerveTranslation, rot, fieldRelative, true);
  }

  public void driveWithGamepad2(Swerve swerve, XboxController driverPad, boolean fieldRelative)
  {
    Translation2d swerveTranslation = new Translation2d( );
    double rot = 0.0;

    double forwardAxis = driverPad.getLeftY( );
    double strafeAxis = driverPad.getLeftX( );

    forwardAxis = Constants.SwerveConstants.invertYAxis ? forwardAxis : -forwardAxis;
    strafeAxis = Constants.SwerveConstants.invertXAxis ? strafeAxis : -strafeAxis;

    Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

    if (Math.abs(tAxes.getNorm( )) < Constants.kStickDeadband)
    {
      swerveTranslation = new Translation2d( );
    }
    else
    {
      // TODO: Clean up this shim from 1678 code
      final double kEpsilon = 1e-12;
      double sin_angle_;
      double cos_angle_;

      double x = tAxes.getX( );
      double y = tAxes.getY( );
      double magnitude = Math.hypot(x, y);

      if (magnitude > kEpsilon)
      {
        sin_angle_ = y / magnitude;
        cos_angle_ = x / magnitude;
      }
      else
      {
        sin_angle_ = 0;
        cos_angle_ = 1;
      }
      double theta_radians = Math.atan2(sin_angle_, cos_angle_);

      Rotation2d deadband_direction = new Rotation2d(theta_radians);
      Translation2d deadband_vector = new Translation2d(deadband_direction.getCos( ) * Constants.kStickDeadband,
          deadband_direction.getSin( ) * Constants.kStickDeadband);

      double scaled_x = tAxes.getX( ) - (deadband_vector.getX( )) / (1 - deadband_vector.getX( ));
      double scaled_y = tAxes.getY( ) - (deadband_vector.getY( )) / (1 - deadband_vector.getY( ));
      swerveTranslation = new Translation2d(scaled_x, scaled_y).times(Constants.SwerveConstants.maxSpeed);
    }

    double rotAxis = driverPad.getRightX( );
    rotAxis = Constants.SwerveConstants.invertRAxis ? rotAxis : -rotAxis;

    if (Math.abs(rotAxis) < Constants.kStickDeadband)
    {
      rot = 0.0;
    }
    else
    {
      rot = Constants.SwerveConstants.maxAngularVelocity * (rotAxis - (Math.signum(rotAxis) * Constants.kStickDeadband))
          / (1 - Constants.kStickDeadband);
    }

    swerve.drive(swerveTranslation, rot, fieldRelative, true);
  }
}
