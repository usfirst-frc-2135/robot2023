// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FCConsts.FCMode;
import frc.robot.Constants.INConsts.INMode;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.SHConsts.SHMode;
import frc.robot.Constants.TCConsts.TCMode;
import frc.robot.Constants.VIConsts.VIRequests;
import frc.robot.commands.Auto1Ball1OppRight;
import frc.robot.commands.Auto1Ball2OppLeft;
import frc.robot.commands.Auto1BallLimelight;
import frc.robot.commands.Auto3BallLeft;
import frc.robot.commands.Auto3BallRight;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoDriveShoot;
import frc.robot.commands.AutoPathSequence;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShootDriveShoot;
import frc.robot.commands.AutoShootLowHub;
import frc.robot.commands.AutoStop;
import frc.robot.commands.DriveLimelight;
import frc.robot.commands.DriveLimelightShoot;
import frc.robot.commands.DriveLimelightStop;
import frc.robot.commands.DriveMotorTest;
import frc.robot.commands.DriveResetSensors;
import frc.robot.commands.DriveSlowMode;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.Dummy;
import frc.robot.commands.ExhaustingAction;
import frc.robot.commands.ExhaustingStop;
import frc.robot.commands.FloorConveyorRun;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.IntakingAction;
import frc.robot.commands.IntakingStop;
import frc.robot.commands.LEDSet;
import frc.robot.commands.RobotInitialize;
import frc.robot.commands.ScoringActionLowerHub;
import frc.robot.commands.ScoringActionUpperHub;
import frc.robot.commands.ScoringPrime;
import frc.robot.commands.ScoringStop;
import frc.robot.commands.ShooterReverse;
import frc.robot.commands.ShooterRun;
import frc.robot.commands.TowerConveyorRun;
import frc.robot.commands.VisionOn;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Power;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TowerConveyor;
import frc.robot.subsystems.Vision;
import frc.robot.team1678.frc2022.controlboard.ControlBoard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
  private static RobotContainer m_robotContainer = new RobotContainer( );

  // Joysticks
  private final XboxController  m_driverPad      = new XboxController(Constants.kDriverPadPort);
  private final XboxController  m_operatorPad    = new XboxController(Constants.kOperatorPadPort);

  // The robot's subsystems
  public final ControlBoard     m_controlBoard   = new ControlBoard(m_driverPad, m_operatorPad);
  public final Swerve           m_swerve         = new Swerve( );
  public final Intake           m_intake         = new Intake( );
  public final FloorConveyor    m_floorConveyor  = new FloorConveyor( );
  public final TowerConveyor    m_towerConveyor  = new TowerConveyor( );
  public final Shooter          m_shooter        = new Shooter( );
  public final Vision           m_vision         = new Vision( );
  public final LED              m_led            = new LED( );
  public final Pneumatics       m_pneumatics     = new Pneumatics( );
  public final Power            m_power          = new Power( );

  // A chooser for autonomous commands
  SendableChooser<Command>      m_chooser        = new SendableChooser<>( );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer( )
  {
    addSmartDashboardWidgets( );

    configureButtonBindings( );

    initDefaultCommands( );

    initAutonomousChooser( );
  }

  public static RobotContainer getInstance( )
  {
    return m_robotContainer;
  }

  private void addSmartDashboardWidgets( )
  {
    // Smartdashboard Subsystems

    // SmartDashboard Buttons
    SmartDashboard.putData("AutoDrive", new AutoDrive(m_swerve, m_intake));
    // SmartDashboard.putData("AutoDrivePath", new AutoDrivePath(m_swerve, "simCurvePath", true));
    SmartDashboard.putData("AutoDriveShoot",
        new AutoDriveShoot(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    SmartDashboard.putData("AutoPathSequence", new AutoPathSequence(m_swerve));
    SmartDashboard.putData("AutoShoot", new AutoShoot(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    SmartDashboard.putData("AutoShootDriveShoot",
        new AutoShootDriveShoot(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    SmartDashboard.putData("AutoShootLowHub",
        new AutoShootLowHub(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    SmartDashboard.putData("AutoStop", new AutoStop(m_swerve));

    SmartDashboard.putData("DriveLimelight", new DriveLimelight(m_swerve, m_vision, false));
    SmartDashboard.putData("DriveLimelightStop",
        new DriveLimelightStop(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    SmartDashboard.putData("DriveLimelightShoot",
        new DriveLimelightShoot(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    SmartDashboard.putData("DriveMotorTest", new DriveMotorTest(m_swerve, true));
    SmartDashboard.putData("DriveResetSensors", new DriveResetSensors(m_swerve));
    SmartDashboard.putData("DriveSlowMode", new DriveSlowMode(m_swerve, false));

    SmartDashboard.putData("ExhaustingAction", new ExhaustingAction(m_intake, m_floorConveyor, m_towerConveyor));
    SmartDashboard.putData("ExhaustingStop", new ExhaustingStop(m_intake, m_floorConveyor, m_towerConveyor));

    SmartDashboard.putData("Fconveyor-STOP", new FloorConveyorRun(m_floorConveyor, FCMode.FCONVEYOR_STOP));
    SmartDashboard.putData("Fconveyor-ACQUIRE", new FloorConveyorRun(m_floorConveyor, FCMode.FCONVEYOR_ACQUIRE));
    SmartDashboard.putData("Fconveyor-EXPEL", new FloorConveyorRun(m_floorConveyor, FCMode.FCONVEYOR_EXPEL));
    SmartDashboard.putData("Fconveyor-EXPELFAST", new FloorConveyorRun(m_floorConveyor, FCMode.FCONVEYOR_EXPEL_FAST));

    SmartDashboard.putData("IntakeDeploy", new IntakeDeploy(m_intake, false));
    SmartDashboard.putData("IntakeStow", new IntakeDeploy(m_intake, false));

    SmartDashboard.putData("Intake-STOP", new IntakeRun(m_intake, INMode.INTAKE_STOP));
    SmartDashboard.putData("Intake-ACQUIRE", new IntakeRun(m_intake, INMode.INTAKE_ACQUIRE));
    SmartDashboard.putData("Intake-EXPEL", new IntakeRun(m_intake, INMode.INTAKE_EXPEL));
    SmartDashboard.putData("IntakingAction", new IntakingAction(m_intake, m_floorConveyor, m_towerConveyor));
    SmartDashboard.putData("IntakingStop", new IntakingStop(m_intake, m_floorConveyor, m_towerConveyor));

    SmartDashboard.putData("LEDSet", new LEDSet(m_led, LEDColor.LEDCOLOR_OFF));
    SmartDashboard.putData("RobotInitialize", new RobotInitialize( ));

    SmartDashboard.putData("ScoringActionUpperHub",
        new ScoringActionUpperHub(m_intake, m_floorConveyor, m_towerConveyor, m_shooter, 2.0));
    SmartDashboard.putData("ScoringActionLowerHub",
        new ScoringActionLowerHub(m_intake, m_floorConveyor, m_towerConveyor, m_shooter, 2.0));
    SmartDashboard.putData("ScoringPrime", new ScoringPrime(m_shooter, m_vision));
    SmartDashboard.putData("ScoringStop", new ScoringStop(m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));

    SmartDashboard.putData("Shooter-OFF", new ShooterRun(m_shooter, SHMode.SHOOTER_STOP));
    SmartDashboard.putData("Shooter-PRIME", new ShooterRun(m_shooter, SHMode.SHOOTER_PRIME));
    SmartDashboard.putData("Shooter-LOW", new ShooterRun(m_shooter, SHMode.SHOOTER_LOWERHUB));
    SmartDashboard.putData("Shooter-HIGH", new ShooterRun(m_shooter, SHMode.SHOOTER_UPPERHUB));
    SmartDashboard.putData("Shooter-REV", new ShooterRun(m_shooter, SHMode.SHOOTER_REVERSE));
    SmartDashboard.putData("ShooterReverse", new ShooterReverse(m_shooter));

    SmartDashboard.putData("Tconveyor-STOP", new TowerConveyorRun(m_towerConveyor, TCMode.TCONVEYOR_STOP));
    SmartDashboard.putData("Tconveyor-ACQUIRE", new TowerConveyorRun(m_towerConveyor, TCMode.TCONVEYOR_ACQUIRE));
    SmartDashboard.putData("Tconveyor-ACQUIRESLOW", new TowerConveyorRun(m_towerConveyor, TCMode.TCONVEYOR_ACQUIRE_SLOW));
    SmartDashboard.putData("Tconveyor-EXPEL", new TowerConveyorRun(m_towerConveyor, TCMode.TCONVEYOR_EXPEL));
    SmartDashboard.putData("Tconveyor-EXPELFAST", new TowerConveyorRun(m_towerConveyor, TCMode.TCONVEYOR_EXPEL_FAST));

    SmartDashboard.putData("Dummy", new Dummy(2135));
  }

  // Create a trigger object that monitors a joystick axis

  private class AxisTrigger extends Trigger
  {
    XboxController m_gamepad;
    Axis           m_axis;

    AxisTrigger(XboxController gamepad, Axis axis)
    {
      m_gamepad = gamepad;
      m_axis = axis;
    }

    @Override
    public boolean getAsBoolean( )
    {
      // This returns whether the trigger is active
      return (m_gamepad.getRawAxis(m_axis.value) > Constants.kTriggerThreshold);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by instantiating
   * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings( )
  {
    ///////////////////////////////////////////////////////
    // Driver Controller Assignments
    // final JoystickButton driverA = new JoystickButton(m_driverPad, XboxController.Button.kA.value);
    // final JoystickButton driverB = new JoystickButton(m_driverPad, XboxController.Button.kB.value);
    // final JoystickButton driverX = new JoystickButton(m_driverPad, XboxController.Button.kX.value);
    // final JoystickButton driverY = new JoystickButton(m_driverPad, XboxController.Button.kY.value);
    //
    final JoystickButton driverLeftBumper = new JoystickButton(m_driverPad, XboxController.Button.kLeftBumper.value);
    final JoystickButton driverRightBumper = new JoystickButton(m_driverPad, XboxController.Button.kRightBumper.value);
    // final JoystickButton driverBack = new JoystickButton(m_driverPad, XboxController.Button.kBack.value);
    final JoystickButton driverStart = new JoystickButton(m_driverPad, XboxController.Button.kStart.value);
    //
    // final POVButton driverUp = new POVButton(m_driverPad, 0);
    // final POVButton driverRight = new POVButton(m_driverPad, 90);
    // final POVButton driverDown = new POVButton(m_driverPad, 180);
    // final POVButton driverLeft = new POVButton(m_driverPad, 270);
    //
    // @formatter:off
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    final AxisTrigger driverLeftTrigger = new AxisTrigger(m_driverPad, XboxController.Axis.kLeftTrigger);
    final AxisTrigger driverRightTrigger = new AxisTrigger(m_driverPad, XboxController.Axis.kRightTrigger);
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    // final AxisTrigger driverLeftTrigger = new AxisTrigger(m_driverPad, XboxController.Axis.kRightX);
    // final AxisTrigger driverRightTrigger = new AxisTrigger(m_driverPad, XboxController.Axis.kRightY);
    // @formatter:on

    // Driver - A, B, X, Y
    // driverA.whenPressed(new Dummy(XboxController.Button.kA.value), true);
    // driverB.whenPressed(new Dummy(XboxController.Button.kB.value), true);
    // driverX.whenPressed(new Dummy(XboxController.Button.kX.value), true);
    // driverY.whenPressed(new Dummy(XboxController.Button.kY.value), true);
    //
    // Driver - Bumpers, start, back
    driverLeftBumper.onTrue(new IntakingAction(m_intake, m_floorConveyor, m_towerConveyor));
    driverLeftBumper.onFalse(new IntakingStop(m_intake, m_floorConveyor, m_towerConveyor));
    driverRightBumper.onTrue(new ScoringActionLowerHub(m_intake, m_floorConveyor, m_towerConveyor, m_shooter, 10.0));
    driverRightBumper.onFalse(new ScoringStop(m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    // driverBack.whenPressed(new Dummy(XboxController.Button.kBack.value), true);
    driverStart.onTrue(new VisionOn(m_vision, VIRequests.VISION_TOGGLE));
    //
    // Driver - POV buttons
    // driverUp.whenPressed(new Dummy(0), true);
    // driverRight.whenPressed(new Dummy(90), true);
    // driverDown.whenPressed(new Dummy(180), true);
    // driverLeft.whenPressed(new Dummy(270), true);
    //
    // Driver - Triggers
    driverLeftTrigger.whenActive(new Dummy(256));
    driverRightTrigger
        .whenActive(new DriveLimelightShoot(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    driverRightTrigger
        .whenInactive(new DriveLimelightStop(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));

    ///////////////////////////////////////////////////////
    // Operator Controller Assignments
    final JoystickButton operA = new JoystickButton(m_operatorPad, XboxController.Button.kA.value);
    final JoystickButton operB = new JoystickButton(m_operatorPad, XboxController.Button.kB.value);
    final JoystickButton operX = new JoystickButton(m_operatorPad, XboxController.Button.kX.value);
    // final JoystickButton operY = new JoystickButton(m_operatorPad, XboxController.Button.kY.value);
    //
    final JoystickButton operLeftBumper = new JoystickButton(m_operatorPad, XboxController.Button.kLeftBumper.value);
    final JoystickButton operRightBumper = new JoystickButton(m_operatorPad, XboxController.Button.kRightBumper.value);
    // final JoystickButton operBack = new JoystickButton(m_operatorPad, XboxController.Button.kBack.value);
    // final JoystickButton operStart = new JoystickButton(m_operatorPad, XboxController.Button.kStart.value);
    //
    // final POVButton operUp = new POVButton(m_operatorPad, 0);
    // final POVButton operRight = new POVButton(m_operatorPad, 90);
    // final POVButton operDown = new POVButton(m_operatorPad, 180);
    // final POVButton operLeft = new POVButton(m_operatorPad, 270);
    //
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // final AxisTrigger operLeftTrigger = new AxisTrigger(m_operatorPad, XboxController.Axis.kLeftTrigger);
    final AxisTrigger operRightTrigger = new AxisTrigger(m_operatorPad, XboxController.Axis.kRightTrigger);
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    // final AxisTrigger operLeftTrigger = new AxisTrigger(m_operatorPad, XboxController.Axis.rightX);
    // final AxisTrigger operRightTrigger = new AxisTrigger(m_operatorPad, XboxController.Axis.rightY);

    // Operator - A, B, X, Y
    operA.onTrue(new IntakeDeploy(m_intake, false));
    operB.onTrue(new ExhaustingAction(m_intake, m_floorConveyor, m_towerConveyor));
    operB.onFalse(new ExhaustingStop(m_intake, m_floorConveyor, m_towerConveyor));
    operX.onFalse(new ScoringStop(m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    // operY.whenPressed(new Dummy(XboxController.Button.kY.value), true);
    //
    // Operator - Bumpers, start, back
    operLeftBumper.onTrue(new IntakingAction(m_intake, m_floorConveyor, m_towerConveyor));
    operLeftBumper.onFalse(new IntakingStop(m_intake, m_floorConveyor, m_towerConveyor));
    operRightBumper.onTrue(new ScoringPrime(m_shooter, m_vision));
    // operBack.whenPressed(new Dummy(XboxController.Button.kBack.value), true);
    // operStart.whenPressed(new Dummy(XboxController.Button.kStart.value), true);
    //
    // Operator - POV buttons
    // operUp.whenPressed(new Dummy(0), true);
    // operRight.whenPressed(new Dummy(90), true);
    // operDown.whenPressed(new Dummy(180), true);
    // operLeft.whenPressed(new Dummy(270), true);
    //
    // Operator Left/Right Trigger
    // operLeftTrigger.whenActive(new Dummy(256));
    operRightTrigger.whileTrue(new RepeatCommand(new ShooterReverse(m_shooter)));
  }

  // Configure the button bindings

  private void initDefaultCommands( )
  {
    // Configure default commands for these subsystems
    m_swerve.setDefaultCommand(new DriveTeleop(m_swerve, m_driverPad));
  }

  private void initAutonomousChooser( )
  {
    // Configure autonomous sendable chooser
    m_chooser.addOption("Auto1Ball1OppRight",
        new Auto1Ball1OppRight(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    m_chooser.addOption("Auto1Ball2OppLeft",
        new Auto1Ball2OppLeft(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    m_chooser.addOption("Auto1BallLimelight",
        new Auto1BallLimelight(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    m_chooser.addOption("Auto3BallLeft",
        new Auto3BallLeft(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    m_chooser.addOption("Auto3BallRight",
        new Auto3BallRight(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    m_chooser.addOption("AutoShootDriveShoot",
        new AutoShootDriveShoot(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    m_chooser.addOption("AutoDriveShoot",
        new AutoDriveShoot(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    m_chooser.setDefaultOption("AutoStop", new AutoStop(m_swerve));

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public XboxController getDriver( )
  {
    return m_driverPad;
  }

  public XboxController getOperator( )
  {
    return m_operatorPad;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand( )
  {
    // The selected command will be run in autonomous
    return m_chooser.getSelected( );
  }
}
