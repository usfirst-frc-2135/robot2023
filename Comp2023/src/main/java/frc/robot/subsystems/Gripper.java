// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GRConsts;
import frc.robot.Constants.GRConsts.GRMode;

public class Gripper extends SubsystemBase
{
  private final WPI_TalonFX mGripper17 = new WPI_TalonFX(GRConsts.kGRPWM17);

  /** Creates a new ExampleSubsystem. */
  public Gripper( )
  {
    setName("Gripper");
    setSubsystem("Gripper");

    mGripper17.setInverted(true);
    mGripper17.setSafetyEnabled(false);
    mGripper17.set(0.0);

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
    double output = 0.0; //off default

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
