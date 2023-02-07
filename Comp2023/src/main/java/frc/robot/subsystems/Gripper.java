//
// Gripper subystem - holds/delivers the cubes and cones
//
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

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
  private final WPI_TalonFX mGripper17 = new WPI_TalonFX(Ports.kCANID_Gripper);

  public Gripper( )
  {
    setName("Gripper");
    setSubsystem("Gripper");

    mGripper17.setInverted(false);
    mGripper17.setSafetyEnabled(false);

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
  }

  public void initialize( )
  {
    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");
    setGripperSpeed(GRMode.GR_STOP);
  }

  public void setGripperSpeed(GRMode mode)
  {
    final String strName;
    double output;

    switch (mode)
    {
      default :
      case GR_STOP :
        strName = "STOP";
        output = 0.0;
        break;
      case GR_ACQUIRE :
        strName = "ACQUIRE";
        output = GRConsts.kGRAcquireSpeed;
        break;
      case GR_EXPEL :
        strName = "EXPEL";
        output = GRConsts.kGRExpelSpeed;
        break;
    }

    DataLogManager.log(getSubsystem( ) + ": Set As - " + strName);
    mGripper17.set(output);
  }
}
