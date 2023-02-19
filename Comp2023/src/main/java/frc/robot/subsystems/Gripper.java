//
// Gripper subystem - holds/delivers the cubes and cones
//
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GRConsts;
import frc.robot.Constants.GRConsts.GRMode;
import frc.robot.Constants.Ports;

//
// Gripper subsystem class
//
public class Gripper extends SubsystemBase
{
  // Member objects
  private final WPI_TalonSRX m_gripper = new WPI_TalonSRX(Ports.kCANID_Gripper);

  // Constructor
  public Gripper( )
  {
    setName("Gripper");
    setSubsystem("Gripper");

    m_gripper.setInverted(true);
    m_gripper.setSafetyEnabled(false);
    m_gripper.set(0.0);

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    if (m_gripper.getStatorCurrent( ) != 0)
      DataLogManager.log("Gripper Current: " + m_gripper.getStatorCurrent( ));
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {
    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");
    setGripperSpeed(GRMode.GR_STOP);
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

    DataLogManager.log(getSubsystem( ) + ": Set As - " + strName);
    m_gripper.set(output);
  }
}
