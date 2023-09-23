package frc.robot.lib.math;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.EXConsts;

public class Conversions
{

  // Gearbox degrees

  /**
   * @param rotations
   *          Input Shaft Rotations
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism
   * @return Output Shaft Degrees of Mechanism
   */
  public static double rotationsToOutputDegrees(double rotations, double gearRatio)
  {
    return rotations * (360.0 / gearRatio);
  }

  /**
   * @param degrees
   *          Output Shaft Degrees of Mechanism
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism
   * @return Input Shaft Rotations
   */
  public static double degreesToInputRotations(double degrees, double gearRatio)
  {
    return degrees / (360.0 / gearRatio);
  }

  // Gearbox radians

  /**
   * @param rotations
   *          Input Shaft Rotations
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism
   * @return Output Shaft Radians of Mechanism
   */
  public static double rotationsToOutputRadians(double rotations, double gearRatio)
  {
    return rotations * (2 * Math.PI / gearRatio);
  }

  /**
   * @param radians
   *          Output Shaft Radians of Mechanism
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism
   * @return Input Shaft Rotations
   */
  public static double radiansToInputRotations(double radians, double gearRatio)
  {
    return radians / (2 * Math.PI / gearRatio);
  }

  // Gearbox velocity

  /**
   * @param rotationsPerSecond
   *          Motor rotations per second
   * @param circumference
   *          Circumference of Wheel
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism (set to 1 for Motor RPM)
   * @return Velocity meters per second
   */
  public static double RPSToMPS(double rotationsPerSecond, double circumference, double gearRatio)
  {
    double wheelRPM = rotationsPerSecond / gearRatio;
    return wheelRPM * circumference;
  }

  /**
   * @param velocity
   *          Velocity meters per second
   * @param circumference
   *          Circumference of Wheel
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism (set to 1 for Motor RPM)
   * @return Motor rotations per second
   */
  public static double MPSToRPS(double velocity, double circumference, double gearRatio)
  {
    double wheelRPM = velocity / circumference;
    return wheelRPM * gearRatio;
  }

  // Gearbox distance

  /**
   * @param rotations
   *          Motor rotations
   * @param circumference
   *          Circumference of Wheel
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism (set to 1 for Motor RPM)
   * @return Distance in meters
   */
  public static double rotationsToMeters(double rotations, double circumference, double gearRatio)
  {
    return rotations * (circumference / gearRatio);
  }

  /**
   * @param distance
   *          Distance in meters
   * @param circumference
   *          Circumference of Wheel
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism (set to 1 for Motor RPM)
   * @return Motor rotations
   */
  public static double metersToRotations(double distance, double circumference, double gearRatio)
  {
    return distance / (circumference / gearRatio);
  }

  // Winch distance

  /**
   * @param meters
   *          Linear winch distance
   * @param rolloutRatio
   *          Winch rollout ratio
   * @return Input Shaft Rotations
   */
  public static double metersToWinchRotations(double meters, double rolloutRatio)
  {
    return meters / Units.inchesToMeters(rolloutRatio);
  }

  public static double inchesToOutputRotations(double inches)
  {
    return inches / EXConsts.kRolloutRatio;
  }

}
