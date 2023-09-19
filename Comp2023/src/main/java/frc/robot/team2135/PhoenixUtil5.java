package frc.robot.team2135;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Falcon500;

public class PhoenixUtil5
{
  private static PhoenixUtil5 instance    = null;
  private static final int    m_retries   = 3;    // Number of version check attempts
  private static final String classString = "PhoenixUtil";

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

    if (errorCode != ErrorCode.OK)
      DataLogManager.log(classString + ": Talon ID " + motor.getDeviceID( ) + " error " + errorCode + " Message: " + message);

    return errorCode;
  }

  public boolean talonSRXInitialize(WPI_TalonSRX talon, String motorName)
  {
    ErrorCode error = ErrorCode.OK;
    int deviceID = 0;
    int fwVersion = 0;
    String baseStr = motorName + " ";
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
            DataLogManager
                .log(classString + ": Talon ID " + deviceID + " " + baseStr + "Incorrect FW version - " + (fwVersion / 256.0));
          break;
        }

        Timer.delay(0.100);
      }
    }

    if (talonValid)
    {
      baseStr += "ver " + (fwVersion / 256.0) + " ";
      // error = talon.configFactoryDefault( ); TODO Clean up later
      //   if (error != ErrorCode.OK)
      //     DataLogManager.log(
      //         classString + ": Talon ID " + deviceID + " error " + error + " " + baseStr + " Message: configFactoryDefault error");
      //   else
      //     initialized = true;
    }

    if (talonValid && initialized)
      DataLogManager.log(classString + ": Talon ID " + deviceID + " error " + error + " " + baseStr + "is INITIALIZED!");
    else
      DataLogManager.log(classString + ": Talon ID " + deviceID + " error " + error + " " + baseStr + "is UNRESPONSIVE!");

    return talonValid && initialized;
  }

}
