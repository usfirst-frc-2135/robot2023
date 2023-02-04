
// ROBOTBUILDER TYPE: Robot.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.util.CTREConfigs;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot
{
  private RobotContainer    m_robotContainer;
  private Command           m_autonomousCommand;

  private boolean           m_faultsCleared = false;

  public static CTREConfigs ctreConfigs;

  /**
   * This function is run when the robot is first started up and should be used for any initialization
   * code.
   */
  @Override
  public void robotInit( )
  {
    // Starts recording to data log
    DataLogManager.start( );
    DataLogManager.log("RobotInit: RoboRIO SN:" + System.getenv("serialnum"));

    ctreConfigs = new CTREConfigs( );

    // Instantiate RobotContainer. Performs button bindings, builds autonomous chooser on the dashboard.
    m_robotContainer = RobotContainer.getInstance( );

    LiveWindow.disableAllTelemetry( );

    CommandScheduler.getInstance( ).onCommandInitialize(cmd -> DataLogManager.log(cmd.getName( ) + ": Init"));
    CommandScheduler.getInstance( ).onCommandInterrupt(cmd -> DataLogManager.log(cmd.getName( ) + ": Interrupted"));
    CommandScheduler.getInstance( ).onCommandFinish(cmd -> DataLogManager.log(cmd.getName( ) + ": End"));

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
    PortForwarder.add(5803, "limelight.local", 5803);
    PortForwarder.add(5804, "limelight.local", 5804);
    PortForwarder.add(5805, "limelight.local", 5805);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic( )
  {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands,
    // running already-scheduled commands, removing finished or interrupted commands, and running 
    // subsystem periodic() methods. This must be called from the robot's periodic block in order
    // for anything in the Command-based framework to work.
    CommandScheduler.getInstance( ).run( );
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit( )
  {
    DataLogManager.log("DisabledInit: Match " + matchTypeToString(DriverStation.getMatchType( )) + DriverStation.getMatchNumber( )
        + ", " + allianceToString(DriverStation.getAlliance( )) + " Alliance");

    // m_robotContainer.m_led.initialize( );
    //m_robotContainer.m_vision.initialize( );

    // These subsystems can use LED and vision subsystems
    m_robotContainer.m_power.initialize( );
    // m_robotContainer.m_pneumatics.initialize( );
    m_robotContainer.m_swerve.initialize( );
  }

  @Override
  public void disabledPeriodic( )
  {
    // If RoboRIO User button is pressed, dump all CAN faults
    if (RobotController.getUserButton( ))
    {
      if (!m_faultsCleared)
      {
        m_faultsCleared = true;
        robotFaultDump( );
      }
    }
    else if (m_faultsCleared)
      m_faultsCleared = false;
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit( )
  {
    DataLogManager.log("AutonomousInit: Match " + matchTypeToString(DriverStation.getMatchType( ))
        + DriverStation.getMatchNumber( ) + ", " + allianceToString(DriverStation.getAlliance( )) + " Alliance");

    m_autonomousCommand = m_robotContainer.getAutonomousCommand( );

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule( );
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic( )
  {}

  @Override
  public void teleopInit( )
  {
    DataLogManager.log("TeleopInit: Match " + matchTypeToString(DriverStation.getMatchType( )) + DriverStation.getMatchNumber( )
        + ", " + allianceToString(DriverStation.getAlliance( )) + " Alliance");

    // This makes sure that the autonomous stops running when teleop starts running. 
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel( );
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic( )
  {}

  @Override
  public void simulationInit( )
  {}

  /**
   * This function is called periodically during simulation.
   */
  @Override
  public void simulationPeriodic( )
  {}

  @Override
  public void testInit( )
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance( ).cancelAll( );
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic( )
  {}

  ///////////////////////////////////////////////////////////////////////////////////////////////////

  private String matchTypeToString(MatchType matchType)
  {
    switch (matchType)
    {
      case None :
        return "N";
      case Practice :
        return "P";
      case Qualification :
        return "Q";
      case Elimination :
        return "E";
    }

    return "<unknown>";
  }

  private String allianceToString(Alliance alliance)
  {
    switch (alliance)
    {
      case Red :
        return "Red";
      case Blue :
        return "Blue";
      case Invalid :
        return "Invalid";
    }

    return "<unknown>";
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////

  private void robotFaultDump( )
  {
    // Print out talon faults and clear sticky ones
    DataLogManager.log("----- DUMP FAULTS --------------");
    // m_robotContainer.m_led.faultDump( );
    //m_robotContainer.m_power.faultDump( );
    // m_robotContainer.m_pneumatics.faultDump( );
    m_robotContainer.m_swerve.faultDump( );
  }
}
