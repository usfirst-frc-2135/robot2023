package frc.robot.lib.util;

public class SwerveModuleConstants
{
  public final int    driveMotorID;
  public final int    steerMotorID;
  public final int    cancoderID;
  public final double steerOffset;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   * 
   * @param driveMotorID
   * @param steerMotorID
   * @param canCoderID
   * @param steerOffset
   */
  public SwerveModuleConstants(int driveMotorID, int steerMotorID, int canCoderID, double steerOffset)
  {
    this.driveMotorID = driveMotorID;
    this.steerMotorID = steerMotorID;
    this.cancoderID = canCoderID;
    this.steerOffset = steerOffset;
  }
}
