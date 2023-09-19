package frc.robot.lib.util;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Falcon500;

public class PhoenixUtil5
{
  private static PhoenixUtil5 instance    = null;
  private static final int    m_retries   = 3;    // Number of version check attempts
  private static final String m_className = "PhoenixUtil5";

  PhoenixUtil5( )
  {}

  public static PhoenixUtil5 getInstance( )
  {
    if (instance == null)
      instance = new PhoenixUtil5( );

    return instance;
  }

  // Talon SRX handler

  public ErrorCode talonSRXCheckError(WPI_TalonSRX motor, String message)
  {
    ErrorCode errorCode = motor.getLastError( );
    int deviceID = motor.getDeviceID( );

    if (errorCode != ErrorCode.OK)
      DataLogManager.log(String.format("%s: ID %2d - Msg: %s - error %d", m_className, deviceID, message, errorCode.value));

    return errorCode;
  }

  public boolean talonSRXInitialize(WPI_TalonSRX talon, String motorName)
  {
    ErrorCode error = ErrorCode.OK;
    int deviceID = 0;
    int fwVersion = 0;
    String baseStr = motorName + " motor: ";
    boolean talonValid = false;
    boolean initialized = false;

    // Display Talon firmware versions
    deviceID = talon.getDeviceID( );
    error = talonSRXCheckError(talon, baseStr + "getDeviceID error");

    Timer.delay(0.250);

    if (error == ErrorCode.OK)
    {
      // This can take multiple attempts before ready
      for (int i = 0; i < m_retries; i++)
      {
        fwVersion = talon.getFirmwareVersion( );
        error = talonSRXCheckError(talon, baseStr + "getFirmwareVersion error");

        if (error == ErrorCode.OK)
        {
          talonValid = true;
          if (fwVersion < Falcon500.kTalonSRXReqVersion)
            DataLogManager.log(String.format("%s: ID %2d - %s - Incorrect FW version: %d - error %d", m_className, deviceID,
                baseStr, (fwVersion / 256.0), error.value));
          break;
        }

        Timer.delay(0.100);
      }
    }

    if (talonValid)
    {
      baseStr += "ver: " + (fwVersion / 256.0) + " ";
      error = talon.configFactoryDefault( );
      if (error != ErrorCode.OK)
        DataLogManager
            .log(String.format("%s: ID %2d - Msg: configFactoryDefault error - %d", m_className, deviceID, error.value));
      else
        initialized = true;
    }

    DataLogManager.log(String.format("%s: ID %2d - %s is %s - error %d", m_className, deviceID, baseStr,
        (talonValid && initialized) ? "INITIALIZED!" : "UNRESPONSIVE!", error.value));

    return talonValid && initialized;
  }

}
