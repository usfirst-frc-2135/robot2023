package frc.robot.lib.math;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Falcon500;

public class Conversions
{

  /**
   * @param counts
   *          Falcon Counts
   * @param gearRatio
   *          Gear Ratio between Falcon and Mechanism
   * @return Degrees of Rotation of Mechanism
   * @deprecated Remove with v6
   */
  @Deprecated // This should go away with v6
  public static double falconToDegrees(double counts, double gearRatio)
  {
    return counts * (360.0 / (gearRatio * 2048.0));
  }

  /**
   * @param degrees
   *          Degrees of rotation of Mechanism
   * @param gearRatio
   *          Gear Ratio between Falcon and Mechanism
   * @return Falcon Counts
   * @deprecated Remove with v6
   */
  @Deprecated // This should go away with v6
  public static double degreesToFalcon(double degrees, double gearRatio)
  {
    double ticks = degrees / (360.0 / (gearRatio * 2048.0));
    return ticks;
  }

  /**
   * @param rotations
   *          Input Shaft Rotations
   * @param gearRatio
   *          Gear Ratio between Falcon and Mechanism
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
   *          Gear Ratio between Falcon and Mechanism
   * @return Input Shaft Rotations
   */
  public static double degreesToInputRotations(double degrees, double gearRatio)
  {
    double rotations = degrees / (360.0 / gearRatio);
    return rotations;
  }

  /**
   * @param rotations
   *          Input Shaft Rotations
   * @param gearRatio
   *          Gear Ratio between Falcon and Mechanism
   * @return Output Shaft Radians of Mechanism
   */
  public static double rotationsToOutputRadians(double rotations, double gearRatio)
  {
    double radians = rotations * (2 * Math.PI / gearRatio);
    return radians;
  }

  /**
   * @param radians
   *          Output Shaft Radians of Mechanism
   * @param gearRatio
   *          Gear Ratio between Falcon and Mechanism
   * @return Input Shaft Rotations
   */
  public static double radiansToInputRotations(double radians, double gearRatio)
  {
    double rotations = radians / (2 * Math.PI / gearRatio);
    return rotations;
  }

  /**
   * @param meters
   *          Linear winch distance
   * @param rolloutRatio
   *          Winch rollout ratio
   * @return Input Shaft Rotations
   */
  public static double metersToInputRotations(double meters, double rolloutRatio)
  {
    double rotations = meters / Units.inchesToMeters(rolloutRatio);
    return rotations;
  }

  /**
   * @param velocityCounts
   *          Falcon Velocity Counts
   * @param gearRatio
   *          Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   * @deprecated Remove with v6
   */
  @Deprecated // This should go away with v6
  public static double falconToRPM(double velocityCounts, double gearRatio)
  {
    double motorRPM = velocityCounts * (600.0 / 2048.0);
    double mechRPM = motorRPM / gearRatio;
    return mechRPM;
  }

  /**
   * @param RPM
   *          RPM of mechanism
   * @param gearRatio
   *          Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   * @deprecated Remove with v6
   */
  @Deprecated // This should go away with v6
  public static double RPMToFalcon(double RPM, double gearRatio)
  {
    double motorRPM = RPM * gearRatio;
    double sensorCounts = motorRPM * (2048.0 / 600.0);
    return sensorCounts;
  }

  /**
   * @param velocitycounts
   *          Falcon Velocity Counts
   * @param circumference
   *          Circumference of Wheel
   * @param gearRatio
   *          Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   * @deprecated Remove with v6
   */
  @Deprecated // This should go away with v6
  public static double falconToMPS(double velocitycounts, double circumference, double gearRatio)
  {
    double wheelRPM = falconToRPM(velocitycounts, gearRatio);
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;
  }

  /**
   * @param velocity
   *          Velocity MPS
   * @param circumference
   *          Circumference of Wheel
   * @param gearRatio
   *          Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   * @deprecated Remove with v6
   */
  @Deprecated // This should go away with v6
  public static double MPSToFalcon(double velocity, double circumference, double gearRatio)
  {
    double wheelRPM = ((velocity * 60) / circumference);
    double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
    return wheelVelocity;
  }

  /**
   * @param distancecounts
   *          Falcon Distance Counts
   * @param circumference
   *          Circumference of Wheel
   * @param gearRatio
   *          Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   * @deprecated Remove with v6
   */
  @Deprecated // This should go away with v6
  public static double falconToMeters(double distanceCounts, double circumference, double gearRatio)
  {
    return distanceCounts * (circumference / Falcon500.kEncoderCPR / gearRatio);
  }

  /**
   * @param distance
   *          distance in meters
   * @param circumference
   *          Circumference of Wheel
   * @param gearRatio
   *          Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   * @deprecated Remove with v6
   */
  @Deprecated // This should go away with v6
  public static double metersToFalcon(double distance, double circumference, double gearRatio)
  {
    return distance / (circumference / Falcon500.kEncoderCPR / gearRatio);
  }

}
