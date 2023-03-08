// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ELConsts.ElbowAngle;
import frc.robot.Constants.EXConsts.ExtensionLength;
import frc.robot.Constants.GRConsts.GRMode;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.VIConsts.VIGoalDirection;
import frc.robot.Constants.WRConsts.WristAngle;
import frc.robot.commands.ArmSetHeightIdle;
import frc.robot.commands.ArmSetHeightScoreHigh;
import frc.robot.commands.ArmSetHeightScoreLow;
import frc.robot.commands.ArmSetHeightScoreMid;
import frc.robot.commands.ArmSetHeightStow;
import frc.robot.commands.AutoChargeStation;
import frc.robot.commands.AutoDriveBalance;
import frc.robot.commands.AutoDrivePath;
import frc.robot.commands.AutoEngageChargeStation;
import frc.robot.commands.AutoPreloadAndEngageChargeStation;
import frc.robot.commands.AutoPreloadAndLeaveCommunity;
import frc.robot.commands.AutoPreloadAndLeaveCommunityPosition3;
import frc.robot.commands.AutoStop;
import frc.robot.commands.DriveLimelightPath;
import frc.robot.commands.DriveSnap;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.Dummy;
import frc.robot.commands.ElbowMoveToAngle;
import frc.robot.commands.ExtensionCalibrate;
import frc.robot.commands.ExtensionMoveToLength;
import frc.robot.commands.GripperRun;
import frc.robot.commands.LEDSet;
import frc.robot.commands.ManualMode;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ResetOdometryToLimelight;
import frc.robot.commands.WristMoveToAngle;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Power;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
  private static RobotContainer    m_instance;

  // Joysticks
  private final XboxController     m_driverPad          = new XboxController(Constants.kDriverPadPort);
  private final XboxController     m_operatorPad        = new XboxController(Constants.kOperatorPadPort);

  // The robot's shared subsystems
  public final LED                 m_led                = new LED( );
  public final Vision              m_vision             = new Vision( );

  // These subsystems can use LED or vision and must be created afterward
  public final Elbow               m_elbow              = new Elbow( );
  public final Extension           m_extension          = new Extension( );
  public final Wrist               m_wrist              = new Wrist( );
  public final Gripper             m_gripper            = new Gripper( );
  public final Power               m_power              = new Power( );
  public final Swerve              m_swerve             = new Swerve( );

  // Commands
  public Command                   m_extensionCalibrate = new ExtensionCalibrate(m_extension);

  // A chooser for autonomous commands
  private SendableChooser<Command> m_autoChooser        = new SendableChooser<>( );

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
    if (m_instance == null)
      m_instance = new RobotContainer( );
    return m_instance;
  }

  /****************************************************************************
   * 
   * Create general dashboard widgets for commands and subsystems
   */
  private void addSmartDashboardWidgets( )
  {
    // SmartDashboard Buttons

    // For future work to set up Shuffleboard layout from code
    // ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");
    // ComplexWidget autoStopEntry = m_autoTab.add("AutoStop", new AutoStop(m_swerve)).withSize(3, 2).withPosition(0, 0);

    // Autonomous buttons  for main routines- chooser order
    SmartDashboard.putData("AutoStop", new AutoStop(m_swerve));
    SmartDashboard.putData("AutoDriveOffCommunity", new AutoDrivePath(m_swerve, "driveOffCommunity", true));
    SmartDashboard.putData("AutoEngageChargeStation", new AutoEngageChargeStation(m_swerve));
    SmartDashboard.putData("AutoChargeStation", new AutoChargeStation(m_swerve));
    SmartDashboard.putData("driveOffCommunityCopy", new AutoDrivePath(m_swerve, "driveOffCommunityCopy", true));
    // SmartDashboard.putData("AutoPreloadAndLeaveCommunity", new AutoPreloadAndLeaveCommunity(m_swerve));
    // SmartDashboard.putData("AutoPreloadAndEngageChargeStation", new AutoPreloadAndEngageChargeStation(m_swerve));
    // SmartDashboard.putData("AutoPreloadAndScoreAnother", new AutoPreloadAndScoreAnother(m_swerve));

    // Autonomous helper commands
    SmartDashboard.putData("AutoDriveBalance", new AutoDriveBalance(m_swerve));

    // SmartDashboard.putData("DriveSlowMode", new DriveSlowMode(m_swerve, false));

    // Path follower tests
    // SmartDashboard.putData("AutoDrivePathForward", new AutoDrivePath(m_swerve, "testForward1m", true));
    // SmartDashboard.putData("AutoDrivePathBackward", new AutoDrivePath(m_swerve, "testBackward1m", true));
    // SmartDashboard.putData("AutoDrivePathForwardLeft", new AutoDrivePath(m_swerve, "testForwardLeft", true));
    // SmartDashboard.putData("AutoDrivePathBackwardRight", new AutoDrivePath(m_swerve, "testBackwardRight", true));
    // SmartDashboard.putData("AutoDrivePathLeft", new AutoDrivePath(m_swerve, "testLeft1m", true));
    // SmartDashboard.putData("AutoDrivePathRight", new AutoDrivePath(m_swerve, "testRight1m", true));
    // SmartDashboard.putData("AutoDrivePathForward2", new AutoDrivePath(m_swerve, "testForward2m", true));
    // SmartDashboard.putData("AutoDrivePathBackward2", new AutoDrivePath(m_swerve, "testBackward2m", true));

    SmartDashboard.putData("ArmSetHeightStow", new ArmSetHeightStow(m_elbow, m_extension, m_wrist));
    SmartDashboard.putData("ArmSetHeightScoreLow", new ArmSetHeightScoreLow(m_elbow, m_extension, m_wrist));
    SmartDashboard.putData("ArmSetHeightScoreMid", new ArmSetHeightScoreMid(m_elbow, m_extension, m_wrist));
    SmartDashboard.putData("ArmSetHeightScoreHigh", new ArmSetHeightScoreHigh(m_elbow, m_extension, m_wrist));

    SmartDashboard.putData("AutoPreloadAndLeaveCommunity",
        new AutoPreloadAndLeaveCommunity(m_swerve, m_elbow, m_extension, m_wrist, m_gripper));
    SmartDashboard.putData("AutoPreloadAndEngageChargeStation",
        new AutoPreloadAndEngageChargeStation(m_swerve, m_elbow, m_extension, m_wrist, m_gripper));
    SmartDashboard.putData("AutoPreloadAndLeaveCommunityPosition3",
        new AutoPreloadAndLeaveCommunityPosition3(m_swerve, m_elbow, m_extension, m_wrist, m_gripper));

    SmartDashboard.putData("LEDSet", new LEDSet(m_led, LEDColor.LEDCOLOR_DASH));
    SmartDashboard.putData("ResetOdometryToLimelight", new ResetOdometryToLimelight(m_swerve, m_vision, 0));

    SmartDashboard.putData("DriveLLLeft", new DriveLimelightPath(m_swerve, m_vision, VIGoalDirection.DIRECTION_LEFT));
    SmartDashboard.putData("DriveLLMiddle", new DriveLimelightPath(m_swerve, m_vision, VIGoalDirection.DIRECTION_MIDDLE));
    SmartDashboard.putData("DriveLLRight", new DriveLimelightPath(m_swerve, m_vision, VIGoalDirection.DIRECTION_RIGHT));
    SmartDashboard.putData("ResetOdo1", new ResetOdometryToLimelight(m_swerve, m_vision, 1));
    SmartDashboard.putData("ResetOdo2", new ResetOdometryToLimelight(m_swerve, m_vision, 2));
    SmartDashboard.putData("ResetOdo3", new ResetOdometryToLimelight(m_swerve, m_vision, 3));
    SmartDashboard.putData("ResetOdo4", new ResetOdometryToLimelight(m_swerve, m_vision, 4));
    SmartDashboard.putData("ResetOdo5", new ResetOdometryToLimelight(m_swerve, m_vision, 5));
    SmartDashboard.putData("ResetOdo6", new ResetOdometryToLimelight(m_swerve, m_vision, 6));
    SmartDashboard.putData("ResetOdo7", new ResetOdometryToLimelight(m_swerve, m_vision, 7));
    SmartDashboard.putData("ResetOdo8", new ResetOdometryToLimelight(m_swerve, m_vision, 8));
    SmartDashboard.putData("DriveLLLeft", new DriveLimelightPath(m_swerve, m_vision, VIGoalDirection.DIRECTION_LEFT));
    SmartDashboard.putData("DriveLLMiddle", new DriveLimelightPath(m_swerve, m_vision, VIGoalDirection.DIRECTION_MIDDLE));
    SmartDashboard.putData("DriveLLRight", new DriveLimelightPath(m_swerve, m_vision, VIGoalDirection.DIRECTION_RIGHT));

    // Elbow subsytem tests
    SmartDashboard.putData("ElbowStow", new ElbowMoveToAngle(m_elbow, ElbowAngle.ELBOW_STOW));
    SmartDashboard.putData("ElbowLow", new ElbowMoveToAngle(m_elbow, ElbowAngle.ELBOW_LOW));
    SmartDashboard.putData("ElbowMid", new ElbowMoveToAngle(m_elbow, ElbowAngle.ELBOW_MID));
    SmartDashboard.putData("ElbowHigh", new ElbowMoveToAngle(m_elbow, ElbowAngle.ELBOW_HIGH));

    // Extension subsytem tests
    SmartDashboard.putData("ExtensionStow", new ExtensionMoveToLength(m_extension, ExtensionLength.EXTENSION_STOW));
    SmartDashboard.putData("ExtensionLow", new ExtensionMoveToLength(m_extension, ExtensionLength.EXTENSION_LOW));
    SmartDashboard.putData("ExtensionMid", new ExtensionMoveToLength(m_extension, ExtensionLength.EXTENSION_MID));
    SmartDashboard.putData("ExtensionHigh", new ExtensionMoveToLength(m_extension, ExtensionLength.EXTENSION_HIGH));

    // Wrist subsytem tests
    SmartDashboard.putData("WristStow", new WristMoveToAngle(m_wrist, WristAngle.WRIST_STOW));
    SmartDashboard.putData("WristLow", new WristMoveToAngle(m_wrist, WristAngle.WRIST_LOW));
    SmartDashboard.putData("WristMid", new WristMoveToAngle(m_wrist, WristAngle.WRIST_MID));
    SmartDashboard.putData("WristHigh", new WristMoveToAngle(m_wrist, WristAngle.WRIST_HIGH));

    // Gripper subsystem test
    SmartDashboard.putData("GripperStop", new GripperRun(m_gripper, GRMode.GR_STOP));
    SmartDashboard.putData("GripperAcquire", new GripperRun(m_gripper, GRMode.GR_ACQUIRE));
    SmartDashboard.putData("GripperHold", new GripperRun(m_gripper, GRMode.GR_HOLD));
    SmartDashboard.putData("GripperExpel", new GripperRun(m_gripper, GRMode.GR_EXPEL));

    // Arm group tests
    SmartDashboard.putData("ArmSetHeightStow", new ArmSetHeightStow(m_elbow, m_extension, m_wrist));
    SmartDashboard.putData("ArmSetHeightLow", new ArmSetHeightScoreLow(m_elbow, m_extension, m_wrist));
    SmartDashboard.putData("ArmSetHeightMid", new ArmSetHeightScoreMid(m_elbow, m_extension, m_wrist));
    SmartDashboard.putData("ArmSetHeightHigh", new ArmSetHeightScoreHigh(m_elbow, m_extension, m_wrist));

    // LED (CANdle) test
    SmartDashboard.putData("LEDSet", new LEDSet(m_led, LEDColor.LEDCOLOR_DASH));

    SmartDashboard.putData("Dummy", new Dummy("2135 is here!")); // Used to test smartdashboard operation
  }

  /****************************************************************************
   * 
   * Use this method to define your button->command mappings. Buttons can be created by instantiating
   * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings( )
  {
    ///////////////////////////////////////////////////////
    //
    // Driver Controller Assignments
    //
    final JoystickButton driverA = new JoystickButton(m_driverPad, XboxController.Button.kA.value);
    final JoystickButton driverB = new JoystickButton(m_driverPad, XboxController.Button.kB.value);
    final JoystickButton driverX = new JoystickButton(m_driverPad, XboxController.Button.kX.value);
    final JoystickButton driverY = new JoystickButton(m_driverPad, XboxController.Button.kY.value);
    //
    final JoystickButton driverLeftBumper = new JoystickButton(m_driverPad, XboxController.Button.kLeftBumper.value);
    final JoystickButton driverRightBumper = new JoystickButton(m_driverPad, XboxController.Button.kRightBumper.value);
    final JoystickButton driverBack = new JoystickButton(m_driverPad, XboxController.Button.kBack.value); // aka View
    final JoystickButton driverStart = new JoystickButton(m_driverPad, XboxController.Button.kStart.value); // aka Menu
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
    driverA.onTrue(new ArmSetHeightIdle(m_elbow, m_extension, m_wrist));
    driverB.onTrue(new DriveLimelightPath(m_swerve, m_vision, VIGoalDirection.DIRECTION_RIGHT));
    driverX.onTrue(new DriveLimelightPath(m_swerve, m_vision, VIGoalDirection.DIRECTION_LEFT));
    driverY.onTrue(new DriveLimelightPath(m_swerve, m_vision, VIGoalDirection.DIRECTION_MIDDLE));
    //
    // Driver - Bumpers, start, back
    driverLeftBumper.onTrue(new Dummy("left bumper"));
    driverRightBumper.onTrue(new GripperRun(m_gripper, GRMode.GR_ACQUIRE));
    driverRightBumper.onFalse(new GripperRun(m_gripper, GRMode.GR_HOLD));
    driverBack.onTrue(new ResetGyro(m_swerve, driverStart, driverBack)); // aka View
    driverStart.onTrue(new ResetGyro(m_swerve, driverStart, driverBack)); // aka Menu
    //
    // Driver - POV buttons
    driverUp.onTrue(new DriveSnap(m_swerve, 0));
    driverRight.onTrue(new DriveSnap(m_swerve, -90));
    driverDown.onTrue(new DriveSnap(m_swerve, 180));
    driverLeft.onTrue(new DriveSnap(m_swerve, 90));
    //
    // Operator Left/Right Trigger
    driverLeftTrigger.onTrue(new Dummy("left trigger"));
    driverRightTrigger.onTrue(new GripperRun(m_gripper, GRMode.GR_EXPEL));
    driverRightTrigger.onFalse(new GripperRun(m_gripper, GRMode.GR_STOP));

    ///////////////////////////////////////////////////////
    //
    // Operator Controller Assignments
    //
    final JoystickButton operA = new JoystickButton(m_operatorPad, XboxController.Button.kA.value);
    final JoystickButton operB = new JoystickButton(m_operatorPad, XboxController.Button.kB.value);
    final JoystickButton operX = new JoystickButton(m_operatorPad, XboxController.Button.kX.value);
    final JoystickButton operY = new JoystickButton(m_operatorPad, XboxController.Button.kY.value);
    //
    final JoystickButton operLeftBumper = new JoystickButton(m_operatorPad, XboxController.Button.kLeftBumper.value);
    final JoystickButton operRightBumper = new JoystickButton(m_operatorPad, XboxController.Button.kRightBumper.value);
    final JoystickButton operBack = new JoystickButton(m_operatorPad, XboxController.Button.kBack.value); // aka View
    final JoystickButton operStart = new JoystickButton(m_operatorPad, XboxController.Button.kStart.value); // aka Menu
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
    operA.toggleOnTrue(new ArmSetHeightScoreLow(m_elbow, m_extension, m_wrist));
    operB.toggleOnTrue(new ArmSetHeightScoreMid(m_elbow, m_extension, m_wrist));
    operX.onTrue(new ArmSetHeightIdle(m_elbow, m_extension, m_wrist));
    operY.onTrue(new ArmSetHeightScoreHigh(m_elbow, m_extension, m_wrist));
    //
    // Operator - Bumpers, start, back
    operLeftBumper.onTrue(new Dummy("left bumper"));
    operRightBumper.onTrue(new GripperRun(m_gripper, GRMode.GR_ACQUIRE));
    operRightBumper.onFalse(new GripperRun(m_gripper, GRMode.GR_HOLD));
    operBack.toggleOnTrue(new ManualMode(m_elbow, m_extension, m_wrist, m_operatorPad)); // aka View

    operStart.onTrue(new Dummy("Start")); // aka Menu
    //
    // Operator - POV buttons
    operUp.onTrue(new Dummy("POV up"));
    operRight.onTrue(new Dummy("POV right"));
    operDown.onTrue(new Dummy("POV down"));
    operLeft.onTrue(new Dummy("POV left"));
    //
    // Operator Left/Right Trigger
    operLeftTrigger.onTrue(new Dummy("left trigger"));
    operRightTrigger.onTrue(new GripperRun(m_gripper, GRMode.GR_EXPEL));
    operRightTrigger.onFalse(new GripperRun(m_gripper, GRMode.GR_STOP));
  }

  /****************************************************************************
   * 
   * Initialize default commands for these subsystems
   */
  private void initDefaultCommands( )
  {
    m_swerve.setDefaultCommand(new DriveTeleop(m_swerve, m_elbow, m_driverPad));
    m_elbow.setDefaultCommand(new ElbowMoveToAngle(m_elbow, ElbowAngle.ELBOW_NOCHANGE));
    m_extension.setDefaultCommand(new ExtensionMoveToLength(m_extension, ExtensionLength.EXTENSION_NOCHANGE));
    m_wrist.setDefaultCommand(new WristMoveToAngle(m_wrist, WristAngle.WRIST_NOCHANGE));
  }

  /****************************************************************************
   * 
   * Set up autonomous chooser
   */
  private void initAutonomousChooser( )
  {
    // Autonomous Chooser
    m_autoChooser.addOption("1 - AutoDriveOffCommunity", new AutoDrivePath(m_swerve, "driveOffCommunity", true));
    m_autoChooser.addOption("2 - AutoDockOnChargeStation", new AutoChargeStation(m_swerve));
    m_autoChooser.addOption("3 - AutoPreloadAndLeaveCommunity",
        new AutoPreloadAndLeaveCommunity(m_swerve, m_elbow, m_extension, m_wrist, m_gripper));
    m_autoChooser.addOption("4 - AutoPreloadAndEngageChargeStation",
        new AutoPreloadAndEngageChargeStation(m_swerve, m_elbow, m_extension, m_wrist, m_gripper));
    m_autoChooser.addOption("5 - AutoPreloadAndEngageChargeStationPosition3",
        new AutoPreloadAndLeaveCommunityPosition3(m_swerve, m_elbow, m_extension, m_wrist, m_gripper));
    // m_chooser.addOption("6 - AutoPreloadAndScoreAnother", new AutoPreloadAndScoreAnother(m_swerve));
    m_autoChooser.setDefaultOption("0 - AutoStop", new AutoStop(m_swerve));

    // Configure autonomous sendable chooser
    SmartDashboard.putData("Auto Mode", m_autoChooser);
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
    return m_autoChooser.getSelected( );
  }
}
