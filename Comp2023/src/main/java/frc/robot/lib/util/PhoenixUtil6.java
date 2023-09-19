package frc.robot.lib.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class PhoenixUtil6
{
  private static PhoenixUtil6 m_instance  = null;
  private static final int    m_retries   = 5;    // Number of version check attempts
  private static final String m_className = "PhoenixUtil6";
  private static double       m_timeout   = 0.100;

  PhoenixUtil6( )
  {}

  public static PhoenixUtil6 getInstance( )
  {
    if (m_instance == null)
      m_instance = new PhoenixUtil6( );

    return m_instance;
  }

  // Talon FX handler

  public boolean talonFXInitialize6(TalonFX motor, String name, TalonFXConfiguration config)
  {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    int deviceID = 0;
    int fwvMajor = 0;
    int fwvMinor = 0;
    int fwvBugfix = 0;
    int fwvBuild = 0;
    boolean talonValid = false;

    // Display Talon firmware versions
    deviceID = motor.getDeviceID( );

    Timer.delay(0.25);

    // This can take multiple attempts before ready
    for (int i = 0; i < m_retries && fwvMajor == 0; i++)
    {
      StatusSignal<Integer> statusSignal = motor.getVersion( );
      status = statusSignal.getError( );
      if (status.isOK( ))
      {
        fwvMajor = (statusSignal.getValue( ) >> 24) & 0xff;
        fwvMinor = (statusSignal.getValue( ) >> 16) & 0xff;
        fwvBugfix = (statusSignal.getValue( ) >> 8) & 0xff;
        fwvBuild = (statusSignal.getValue( ) >> 0) & 0xff;
      }
      else
        Timer.delay(0.1);
    }

    talonValid = (fwvMajor >= Constants.kPhoenixMajorVersion);

    if (config != null)
      if ((status = motor.getConfigurator( ).apply(config, m_timeout)) != StatusCode.OK)
        DataLogManager.log(String.format("%s: ID %2d - %s motor: getConfigurator.apply - %s!", m_className, deviceID, name,
            status.getDescription( )));

    if ((status = motor.setControl(new VoltageOut(0).withEnableFOC(false))) != StatusCode.OK)
      DataLogManager
          .log(String.format("%s: ID %2d - %s motor: setControl - %s!", m_className, deviceID, name, status.getDescription( )));

    // Configure sensor settings
    if ((status = motor.setRotorPosition(0.0)) != StatusCode.OK)
      DataLogManager.log(
          String.format("%s: ID %2d - %s motor: setRotorPosition - %s!", m_className, deviceID, name, status.getDescription( )));

    motor.setSafetyEnabled(false);

    DataLogManager.log(String.format("%s: ID %2d - %s motor:    ver: %d.%d.%d.%d is %s!", m_className, deviceID, name, fwvMajor,
        fwvMinor, fwvBugfix, fwvBuild, (talonValid) ? "VALID" : "URESPONSIVE"));

    return talonValid;
  }

  // CANCoder handler

  public boolean canCoderInitialize6(CANcoder canCoder, String name, CANcoderConfiguration config)
  {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    int deviceID = 0;
    int fwvMajor = 0;
    int fwvMinor = 0;
    int fwvBugfix = 0;
    int fwvBuild = 0;
    boolean canCoderValid = false;

    // Display Talon firmware versions
    deviceID = canCoder.getDeviceID( );

    // This can take multiple attempts before ready
    for (int i = 0; i < m_retries && fwvMajor == 0; i++)
    {
      StatusSignal<Integer> statusSignal = canCoder.getVersion( );
      status = statusSignal.getError( );
      if (status.isOK( ))
      {
        fwvMajor = (statusSignal.getValue( ) >> 24) & 0xff;
        fwvMinor = (statusSignal.getValue( ) >> 16) & 0xff;
        fwvBugfix = (statusSignal.getValue( ) >> 8) & 0xff;
        fwvBuild = (statusSignal.getValue( ) >> 0) & 0xff;
      }
      else
        Timer.delay(0.1);
    }

    canCoderValid = (fwvMajor >= Constants.kPhoenixMajorVersion);

    if (config != null)
      if ((status = canCoder.getConfigurator( ).apply(config, m_timeout)) != StatusCode.OK)
        DataLogManager.log(String.format("%s: ID %2d - %s CANCoder: getConfigurator.apply - %s!", m_className, deviceID, name,
            status.getDescription( )));

    DataLogManager.log(String.format("%s: ID %2d - %s CANCoder: ver: %d.%d.%d.%d is %s!", m_className, deviceID, name, fwvMajor,
        fwvMinor, fwvBugfix, fwvBuild, (canCoderValid) ? "VALID" : "URESPONSIVE"));

    return canCoderValid;
  }

  //   // Pigeon IMU handler

  public boolean pigeon2Initialize6(Pigeon2 pigeon2, Pigeon2Configuration config)
  {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    int deviceID = 0;
    int fwvMajor = 0;
    int fwvMinor = 0;
    int fwvBugfix = 0;
    int fwvBuild = 0;
    boolean pigeon2Valid = false;

    // Display Talon firmware versions
    deviceID = pigeon2.getDeviceID( );

    Timer.delay(0.25);

    // This can take multiple attempts before ready
    for (int i = 0; i < m_retries && fwvMajor == 0; i++)
    {
      StatusSignal<Integer> statusSignal = pigeon2.getVersion( );
      status = statusSignal.getError( );
      if (status.isOK( ))
      {
        fwvMajor = (statusSignal.getValue( ) >> 24) & 0xff;
        fwvMinor = (statusSignal.getValue( ) >> 16) & 0xff;
        fwvBugfix = (statusSignal.getValue( ) >> 8) & 0xff;
        fwvBuild = (statusSignal.getValue( ) >> 0) & 0xff;
      }
      else
        Timer.delay(0.1);
    }

    pigeon2Valid = (fwvMajor >= Constants.kPhoenixMajorVersion);

    if (config != null)
      if ((status = pigeon2.getConfigurator( ).apply(config, m_timeout)) != StatusCode.OK)
        DataLogManager.log(
            String.format("%s: ID %2d - pigeon2: getConfigurator.apply - %s!", m_className, deviceID, status.getDescription( )));

    // Configure sensor settings
    if ((status = pigeon2.setYaw(0.0)) != StatusCode.OK)
      DataLogManager.log(String.format("%s: ID %2d - pigeon2: setYaw - %s!", m_className, deviceID, status.getDescription( )));

    DataLogManager.log(String.format("%s: ID %2d - pigeon2:    ver: %d.%d.%d.%d is %s!", m_className, deviceID, fwvMajor,
        fwvMinor, fwvBugfix, fwvBuild, (pigeon2Valid) ? "VALID" : "URESPONSIVE"));

    return pigeon2Valid;
  }
}
