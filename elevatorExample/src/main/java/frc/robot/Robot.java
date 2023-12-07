// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ExampleSmartMotorController.PIDMode;

public class Robot extends TimedRobot
{
  private final static double                       kDt           = 0.020;  // 20 msec per RoboRIO loop

  private final static double                       kv            = 2.0;    // Max velocity - RPS
  private final static double                       ka            = 8.0;    // Max acceleration - RPS^2

  private final static double                       goal_1        = 0.0;  // Goal 1 position
  private final static double                       goal_2        = 0.5;  // Goal 2 position

  private final static XboxController               m_controller  = new XboxController(1);
  private final static ExampleSmartMotorController  m_motor       = new ExampleSmartMotorController(5);
  // Note: These gains are fake, and will have to be tuned for your robot.
  // private final SimpleMotorFeedforward              m_feedforward = new SimpleMotorFeedforward(ks, kf);

  private final static TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kv, ka);
  private TrapezoidProfile.State                    m_goal        = new TrapezoidProfile.State( );
  private TrapezoidProfile.State                    m_setpoint    = new TrapezoidProfile.State( );

  @Override
  public void robotInit( )
  {}

  @Override
  public void teleopPeriodic( )
  {
    if (m_controller.getAButton( ))
      m_goal = new TrapezoidProfile.State(goal_1, 0);
    else if (m_controller.getBButton( ))
      m_goal = new TrapezoidProfile.State(goal_2, 0);
    SmartDashboard.putNumber("0-goal", m_goal.position);

    // Create a motion profile with the given maximum velocity and maximum
    // acceleration constraints for the next setpoint, the desired goal, and the
    // current setpoint.
    TrapezoidProfile profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = profile.calculate(kDt);

    // Send setpoint to offboard controller PID
    m_motor.setSetpoint(PIDMode.kPosition, m_setpoint.position, 0.0);
    SmartDashboard.putNumber("1-setpoint", m_setpoint.position);
  }

  @Override
  public void robotPeriodic( )
  {
    m_motor.periodic( );
  }

}
