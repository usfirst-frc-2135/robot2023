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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.Dummy;
import frc.robot.subsystems.Power;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
  private static RobotContainer m_robotContainer;

  // Joysticks
  private final XboxController  m_driverPad   = new XboxController(Constants.kDriverPadPort);
  private final XboxController  m_operatorPad = new XboxController(Constants.kOperatorPadPort);

  // The robot's subsystems
  // public final LED              m_led           = new LED( );
  public final Power            m_power       = new Power( );
  // public final Pneumatics       m_pneumatics    = new Pneumatics( );
  // public final Vision           m_vision        = new Vision( );

  // These subsystems can use LED or vision and must be created afterward
  public final Swerve           m_swerve      = new Swerve( );

  // A chooser for autonomous commands
  SendableChooser<Command>      m_chooser     = new SendableChooser<>( );

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
    // Smartdashboard Subsystems

    // SmartDashboard Buttons
    
    // SmartDashboard.putData("DriveLimelight", new DriveLimelight(m_swerve, m_vision, false));
    // SmartDashboard.putData("DriveLimelightStop",
    //     new DriveLimelightStop(m_swerve, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    // SmartDashboard.putData("DriveMotorTest", new DriveMotorTest(m_swerve, true));
    // SmartDashboard.putData("DriveResetSensors", new DriveResetSensors(m_swerve));
    // SmartDashboard.putData("DriveSlowMode", new DriveSlowMode(m_swerve, false));

    // SmartDashboard.putData("LEDSet", new LEDSet(m_led, LEDColor.LEDCOLOR_OFF));
    // SmartDashboard.putData("RobotInitialize", new RobotInitialize( ));

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
    // // driverBack.whenPressed(new Dummy(XboxController.Button.kBack.value), true);
    // driverStart.onTrue(new VisionOn(m_vision, VIRequests.VISION_TOGGLE));
    //
    // Driver - POV buttons
    // driverUp.whenPressed(new Dummy(0), true);
    // driverRight.whenPressed(new Dummy(90), true);
    // driverDown.whenPressed(new Dummy(180), true);
    // driverLeft.whenPressed(new Dummy(270), true);
    //
    // Driver - Triggers
    driverLeftTrigger.whenActive(new Dummy(256));
    
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
    // operA.onTrue(new IntakeDeploy(m_intake, false));
    // operB.onTrue(new ExhaustingAction(m_intake, m_floorConveyor, m_towerConveyor));
    // operB.onFalse(new ExhaustingStop(m_intake, m_floorConveyor, m_towerConveyor));
    // operX.onFalse(new ScoringStop(m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    // // operY.whenPressed(new Dummy(XboxController.Button.kY.value), true);
    // //
    // // Operator - Bumpers, start, back
    // operLeftBumper.onTrue(new IntakingAction(m_intake, m_floorConveyor, m_towerConveyor));
    // operLeftBumper.onFalse(new IntakingStop(m_intake, m_floorConveyor, m_towerConveyor));
    // operRightBumper.onTrue(new ScoringPrime(m_shooter, m_vision));
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
    // operRightTrigger.whileTrue(new RepeatCommand(new ShooterReverse(m_shooter)));
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

    // SmartDashboard.putData("Auto Mode", m_chooser);
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
