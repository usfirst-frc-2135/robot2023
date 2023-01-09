
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class SimulateLimelight extends CommandBase
{
  private final double LL2_HORIZ_HALF_FOV_DEG = 29.8;   // 59.6 degrees FOV
  private final double LL2_VERT_HALF_FOV_DEG  = 24.85;  // 49.7 degrees FOV

  Swerve               m_swerve;
  Translation2d        m_goalTranslation;
  double               m_goalHeightMeters;
  Translation2d        m_cameraTranslationOffset;
  Rotation2d           m_cameraYawOffset;
  double               m_cameraHeightMeters;
  double               m_cameraPitchRadians;

  Mechanism2d          m_mech                 = new Mechanism2d(2.4, 2.0, new Color8Bit(Color.kBlack));
  MechanismRoot2d      m_mechRoot             = m_mech.getRoot("Limelight", 0, 0);
  MechanismLigament2d  m_mechLigament         =
      m_mechRoot.append(new MechanismLigament2d("Target", 0.1, 0.0, 16, new Color8Bit(Color.kGreen)));

  public SimulateLimelight(Swerve swerve, Translation2d goalTranslation, double goalHeightMeters,
      Translation2d cameraTranslationOffset, Rotation2d cameraYawOffset, double cameraHeightMeters, double cameraPitchRadians)
  {
    m_swerve = swerve;
    m_goalTranslation = goalTranslation;
    m_goalHeightMeters = goalHeightMeters;
    m_cameraTranslationOffset = cameraTranslationOffset;
    m_cameraYawOffset = cameraYawOffset;
    m_cameraHeightMeters = cameraHeightMeters;
    m_cameraPitchRadians = cameraPitchRadians;

    setName("SimulateLimelight");
    SmartDashboard.putData("Limelight (Sim)", m_mech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    Pose2d robotPose = m_swerve.getPose( );
    // The camera may not be centered on the robot, so we need to account for offets
    Pose2d cameraPose = new Pose2d(robotPose.getTranslation( ).plus(m_cameraTranslationOffset.rotateBy(robotPose.getRotation( ))),
        robotPose.getRotation( ).plus(m_cameraYawOffset));

    // Get the goal translation and height as if the camera were at [0, 0, 0]
    Translation2d cameraRelativeGoalTranslation =
        (m_goalTranslation.minus(cameraPose.getTranslation( )).rotateBy(cameraPose.getRotation( ).unaryMinus( )));
    double cameraRelativeGoalHeightMeters = m_goalHeightMeters - m_cameraHeightMeters;

    double tx = Units.radiansToDegrees(-Math.atan2(cameraRelativeGoalTranslation.getY( ), cameraRelativeGoalTranslation.getX( )));
    double ty = Units.radiansToDegrees(
        Math.atan2(cameraRelativeGoalHeightMeters, cameraRelativeGoalTranslation.getNorm( )) - m_cameraPitchRadians);

    NetworkTable table = NetworkTableInstance.getDefault( ).getTable("limelight");
    table.getEntry("tx").setValue(tx);
    table.getEntry("ty").setValue(ty);
    table.getEntry("tv").setValue(((tx >= -LL2_HORIZ_HALF_FOV_DEG && tx <= LL2_HORIZ_HALF_FOV_DEG)
        && (ty >= -LL2_VERT_HALF_FOV_DEG && ty <= LL2_VERT_HALF_FOV_DEG)) ? 1.0 : 0.0);
    m_mechRoot.setPosition(tx / LL2_HORIZ_HALF_FOV_DEG + (1.2 - 0.05), ty / LL2_VERT_HALF_FOV_DEG + 1);
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
    return true;
  }
}
