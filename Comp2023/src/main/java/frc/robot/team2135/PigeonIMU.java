package frc.robot.team2135;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SWConsts;

public class PigeonIMU
{
  // Pigeon 2 object
  private Pigeon2    m_gyro;

  // Offset adjustments to each axis to hide gyro hardware offsets
  private Rotation2d m_yawAdjustment   = new Rotation2d( );
  private Rotation2d m_pitchAdjustment = new Rotation2d( );
  private Rotation2d m_rollAdjustment  = new Rotation2d( );

  /**
   * Construct the pigeon2 IMU using the CAN ID and CAN bus.
   *
   * @param port
   *          CAN ID value
   * @param bus
   *          CAN bus name
   */
  public PigeonIMU(int canID, String bus)
  {
    m_gyro = new Pigeon2(canID, bus);
    PhoenixUtil6.getInstance( ).pigeon2Initialize6(m_gyro, null);

    // Eliminate all initial offsets
    setYaw(0.0);
    setPitch(0.0);
    setRoll(0.0);
  }

  /**
   * Gets the unadjusted raw yaw from the gyro.
   *
   * @return Yaw as a rotation2d
   */
  private Rotation2d getUnadjustedYaw( )
  {
    return Rotation2d.fromDegrees(m_gyro.getYaw( ).getValue( ));
  }

  /**
   * Gets the unadjusted raw pitch from the gyro.
   *
   * @return Pitch as a rotation2d
   */
  private Rotation2d getUnadjustedPitch( )
  {
    return Rotation2d.fromDegrees(m_gyro.getPitch( ).getValue( ));
  }

  /**
   * Gets the unadjusted raw roll from the gyro.
   *
   * @return Roll as a rotation2d
   */
  private Rotation2d getUnadjustedRoll( )
  {
    return Rotation2d.fromDegrees(m_gyro.getRoll( ).getValue( ));
  }

  /**
   * Gets the robot yaw adjusted by the raw gyro offset.
   *
   * @return Yaw as a rotation2d
   */
  public Rotation2d getYaw( )
  {
    Rotation2d rot2d = getUnadjustedYaw( ).rotateBy(m_yawAdjustment.unaryMinus( ));

    return (SWConsts.gyroInvert) ? rot2d.unaryMinus( ) : rot2d;
  }

  /**
   * Gets the robot pitch adjusted by the raw gyro offset.
   *
   * @return Pitch as a rotation2d
   */
  public Rotation2d getPitch( )
  {
    Rotation2d rot2d = getUnadjustedPitch( ).rotateBy(m_pitchAdjustment.unaryMinus( ));

    return rot2d;
  }

  /**
   * Gets the robot roll adjusted by the raw gyro offset.
   *
   * @return Roll as a rotation2d
   */
  public Rotation2d getRoll( )
  {
    return getUnadjustedRoll( ).rotateBy(m_rollAdjustment.unaryMinus( ));
  }

  /**
   * Sets the yaw register to read the specified value.
   *
   * @param angleDeg
   *          New yaw in degrees
   */
  public void setYaw(double angleDeg)
  {
    m_yawAdjustment = getUnadjustedYaw( ).rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus( ));
  }

  /**
   * Sets the pitch register to read the specified value.
   *
   * @param angleDeg
   *          New pitch in degrees
   */
  public void setPitch(double angleDeg)
  {
    m_pitchAdjustment = getUnadjustedPitch( ).rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus( ));
  }

  /**
   * Sets the roll register to read the specified value.
   *
   * @param angleDeg
   *          New roll in degrees
   */
  public void setRoll(double angleDeg)
  {
    m_rollAdjustment = getUnadjustedRoll( ).rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus( ));
  }

}
