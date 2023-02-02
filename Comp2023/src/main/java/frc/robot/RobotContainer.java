// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GRConsts.GRMode;
import frc.robot.commands.AutoDrivePath;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.Dummy;
import frc.robot.commands.GripperRun;
import frc.robot.commands.ResetGyro;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Power;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

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

  // The robot's shared subsystems
  // public final LED              m_led         = new LED( );
  public final Vision           m_vision      = new Vision( );

  // These subsystems can use LED or vision and must be created afterward
  // public final Arm              m_arm         = new Arm();
  public final Gripper          m_Gripper     = new Gripper( );
  // public final Pneumatics       m_pneumatics  = new Pneumatics( );
  public final Power            m_power       = new Power( );
  public final Swerve           m_swerve      = new Swerve( );

  // A chooser for autonomous commands
  SendableChooser<Command>      m_chooser        = new SendableChooser<>( );

  //Field 2d widget declaration
  public final Field2d          m_field2d        = new Field2d( );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer( )
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
    // SmartDashboard Buttons
    //SmartDashboard.putData("AutoDrivePathForward", new AutoDrivePath(m_swerve, "forward1m", true));

    // SmartDashboard.putData("DriveLimelight", new DriveLimelight(m_swerve, m_vision, false));
    // SmartDashboard.putData("DriveLimelightStop",
    //     new DriveLimelightStop(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    // SmartDashboard.putData("DriveMotorTest", new DriveMotorTest(m_swerve, true));
    // SmartDashboard.putData("DriveResetSensors", new DriveResetSensors(m_swerve));
    // SmartDashboard.putData("DriveSlowMode", new DriveSlowMode(m_swerve, false));
    // SmartDashboard.putData("LEDSet", new LEDSet(m_led, LEDColor.LEDCOLOR_OFF));

    SmartDashboard.putData("Field2d", m_field2d);
    SmartDashboard.putData("Dummy", new Dummy(2135));
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
    final JoystickButton driverA = new JoystickButton(m_driverPad, XboxController.Button.kA.value);
    final JoystickButton driverB = new JoystickButton(m_driverPad, XboxController.Button.kB.value);
    final JoystickButton driverX = new JoystickButton(m_driverPad, XboxController.Button.kX.value);
    final JoystickButton driverY = new JoystickButton(m_driverPad, XboxController.Button.kY.value);
    //
    final JoystickButton driverLeftBumper = new JoystickButton(m_driverPad, XboxController.Button.kLeftBumper.value);
    final JoystickButton driverRightBumper = new JoystickButton(m_driverPad, XboxController.Button.kRightBumper.value);
    final JoystickButton driverBack = new JoystickButton(m_driverPad, XboxController.Button.kBack.value);
    final JoystickButton driverStart = new JoystickButton(m_driverPad, XboxController.Button.kStart.value);
    //
    final POVButton driverUp = new POVButton(m_driverPad, 0);
    final POVButton driverRight = new POVButton(m_driverPad, 90);
    final POVButton driverDown = new POVButton(m_driverPad, 180);
    final POVButton driverLeft = new POVButton(m_driverPad, 270);
    //
    // @formatter:off
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    final Trigger driverLeftTrigger = new Trigger(( )->m_driverPad.getLeftTriggerAxis() > Constants.kTriggerThreshold);
    final Trigger driverRightTrigger = new Trigger(( )->m_driverPad.getRightTriggerAxis() > Constants.kTriggerThreshold);
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    // final Trigger driverLeftTrigger = new Trigger(( )->m_driverPad.getRightX() > Constants.kTriggerThreshold);
    // final Trigger driverRightTrigger = new Trigger(( )->m_driverPad.getRightY() > Constants.kTriggerThreshold);
    // @formatter:on

    // Driver - A, B, X, Y
    driverA.onTrue(new Dummy(XboxController.Button.kA.value));
    driverB.onTrue(new Dummy(XboxController.Button.kB.value));
    driverX.onTrue(new Dummy(XboxController.Button.kX.value));
    driverY.onTrue(new Dummy(XboxController.Button.kY.value));
    //
    // Driver - Bumpers, start, back
    driverLeftBumper.onTrue(new Dummy(XboxController.Button.kLeftBumper.value));
    driverRightBumper.onTrue(new Dummy(XboxController.Button.kRightBumper.value));
    driverBack.onTrue(new ResetGyro(m_swerve, driverStart, driverBack));
    driverStart.onTrue(new ResetGyro(m_swerve, driverStart, driverBack));
    //
    // Driver - POV buttons
    driverUp.onTrue(new Dummy(0));
    driverRight.onTrue(new Dummy(90));
    driverDown.onTrue(new Dummy(180));
    driverLeft.onTrue(new Dummy(270));
    //
    // Operator Left/Right Trigger
    driverLeftTrigger.onTrue(new Dummy(128));
    driverRightTrigger.onTrue(new Dummy(129));

    ///////////////////////////////////////////////////////
    // Operator Controller Assignments
    final JoystickButton operA = new JoystickButton(m_operatorPad, XboxController.Button.kA.value);
    final JoystickButton operB = new JoystickButton(m_operatorPad, XboxController.Button.kB.value);
    final JoystickButton operX = new JoystickButton(m_operatorPad, XboxController.Button.kX.value);
    final JoystickButton operY = new JoystickButton(m_operatorPad, XboxController.Button.kY.value);
    //
    final JoystickButton operLeftBumper = new JoystickButton(m_operatorPad, XboxController.Button.kLeftBumper.value);
    final JoystickButton operRightBumper = new JoystickButton(m_operatorPad, XboxController.Button.kRightBumper.value);
    final JoystickButton operBack = new JoystickButton(m_operatorPad, XboxController.Button.kBack.value);
    final JoystickButton operStart = new JoystickButton(m_operatorPad, XboxController.Button.kStart.value);
    //
    final POVButton operUp = new POVButton(m_operatorPad, 0);
    final POVButton operRight = new POVButton(m_operatorPad, 90);
    final POVButton operDown = new POVButton(m_operatorPad, 180);
    final POVButton operLeft = new POVButton(m_operatorPad, 270);
    //
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    final Trigger operLeftTrigger = new Trigger(( ) -> m_operatorPad.getLeftTriggerAxis( ) > Constants.kTriggerThreshold);
    final Trigger operRightTrigger = new Trigger(( ) -> m_operatorPad.getRightTriggerAxis( ) > Constants.kTriggerThreshold);
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    // final Trigger operLeftTrigger = new Trigger(( ) -> m_operatorPad.getRightX( ) > Constants.kTriggerThreshold);
    // final Trigger operRightTrigger = new Trigger(( ) -> m_operatorPad.getRightY( ) > Constants.kTriggerThreshold);

    // Operator - A, B, X, Y
    operA.onTrue(new Dummy(XboxController.Button.kA.value));
    operB.onTrue(new Dummy(XboxController.Button.kB.value));
    operX.onTrue(new Dummy(XboxController.Button.kX.value));
    operY.onTrue(new Dummy(XboxController.Button.kY.value));
    //
    // Operator - Bumpers, start, back
    operLeftBumper.onTrue(new GripperRun(m_Gripper, GRMode.GR_ACQUIRE));
    operLeftBumper.onFalse(new GripperRun(m_Gripper, GRMode.GR_STOP));
    operRightBumper.onTrue(new GripperRun(m_Gripper, GRMode.GR_EXPEL));
    operRightBumper.onFalse(new GripperRun(m_Gripper, GRMode.GR_STOP));
    operBack.onTrue(new Dummy(XboxController.Button.kBack.value));
    operStart.onTrue(new Dummy(XboxController.Button.kStart.value));
    //
    // Operator - POV buttons
    operUp.onTrue(new Dummy(0));
    operRight.onTrue(new Dummy(90));
    operDown.onTrue(new Dummy(180));
    operLeft.onTrue(new Dummy(270));
    //
    // Operator Left/Right Trigger
    operLeftTrigger.onTrue(new Dummy(130));
    operRightTrigger.onTrue(new Dummy(131));
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
    m_chooser.setDefaultOption("PrintMe", new PrintCommand("Auto PrintMe Command"));
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
