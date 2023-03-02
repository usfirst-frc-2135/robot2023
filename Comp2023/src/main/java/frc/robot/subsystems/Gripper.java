//
// Gripper subystem - holds/delivers the cubes and cones
//
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GRConsts;
import frc.robot.Constants.GRConsts.GRMode;
import frc.robot.Constants.Ports;
import frc.robot.team2135.PhoenixUtil;

//
// Gripper subsystem class
//
public class Gripper extends SubsystemBase
{
  // Member objects
  private final WPI_TalonSRX              m_gripper             = new WPI_TalonSRX(Ports.kCANID_Gripper);

  private boolean                         m_gripperValid;                // Health indicator for gripper Talon
  private boolean                         m_gripperDebug        = false; // Enable/disable dashboard and logging updates

  //Devices and simulation objs
  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true,
      GRConsts.kSupplyCurrentLimit, GRConsts.kSupplyTriggerCurrent, GRConsts.kSupplyTriggerTime);

  // Constructor
  public Gripper( )
  {
    setName("Gripper");
    setSubsystem("Gripper");

    m_gripperValid = PhoenixUtil.getInstance( ).talonSRXInitialize(m_gripper, "gripper");

    SmartDashboard.putBoolean("HL_validGR", m_gripperValid);

    gripperTalonInitialize(m_gripper, GRConsts.kInvertMotor);

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    double currentDraw = m_gripper.getStatorCurrent( );

    if (m_gripperDebug && (currentDraw >= Constants.kMinCurrent))
      DataLogManager.log(String.format("%s: current: %.1f", getSubsystem( ), m_gripper.getStatorCurrent( )));
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {
    DataLogManager.log(getSubsystem( ) + ": Subsystem initialized!");
    setGripperSpeed(GRMode.GR_STOP);
  }

  private void gripperTalonInitialize(WPI_TalonSRX motor, boolean inverted)
  {
    motor.setInverted(inverted);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setInverted");
    motor.setNeutralMode(NeutralMode.Coast);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setNeutralMode");
    motor.setSafetyEnabled(false);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setSafetyEnabled");

    motor.configVoltageCompSaturation(12.0);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configVoltageCompSaturation");
    motor.enableVoltageCompensation(true);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "enableVoltageCompensation");

    motor.configSupplyCurrentLimit(m_supplyCurrentLimits);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configSupplyCurrentLimits");

    motor.set(ControlMode.PercentOutput, 0.0);
  }

  public void setGripperSpeed(GRMode mode)
  {
    final String strName;
    double output = 0.0; // default: off

    switch (mode)
    {
      default :
      case GR_STOP :
        strName = "STOP";
        output = 0.0;
        break;
      case GR_ACQUIRE :
        strName = "ACQUIRE";
        output = GRConsts.kGripperSpeedAcquire;
        break;
      case GR_EXPEL :
        strName = "EXPEL";
        output = GRConsts.kGripperSpeedExpel;
        break;
      case GR_HOLD :
        strName = "HOLD";
        output = GRConsts.kGripperSpeedHold;
        break;
    }

    DataLogManager.log(getSubsystem( ) + ": Mode is now - " + strName);
    m_gripper.set(output);
  }
}
