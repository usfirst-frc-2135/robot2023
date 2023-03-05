package frc.robot.team2135;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_Faults;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Falcon500;

public class PhoenixUtil
{
  private static PhoenixUtil  instance    = null;
  private static final int    m_retries   = 4;    // Number of version check attempts
  private static final String classString = "PhoenixUtil";

  PhoenixUtil( )
  {}

  public static PhoenixUtil getInstance( )
  {
    if (instance == null)
      instance = new PhoenixUtil( );

    return instance;
  }

  // Talon FX and SRX handlers

  public ErrorCode checkTalonError(BaseMotorController talon, String message)
  {
    ErrorCode errorCode = talon.getLastError( );

    if (errorCode != ErrorCode.OK)
      DataLogManager.log(classString + ": Talon ID " + talon.getDeviceID( ) + " error " + errorCode + " Message: " + message);

    return errorCode;
  }

  private boolean talonInitialize(BaseMotorController talon, String motorName)
  {
    ErrorCode error = ErrorCode.OK;
    int deviceID = 0;
    int fwVersion = 0;
    String baseStr = motorName + " ";
    boolean talonValid = false;
    boolean initialized = false;

    // Display Talon firmware versions
    deviceID = talon.getDeviceID( );
    error = checkTalonError(talon, baseStr + "getDeviceID error");

    Timer.delay(0.250);

    if (error == ErrorCode.OK)
    {
      // This can take multiple attempts before ready
      for (int i = 0; i < m_retries; i++)
      {
        fwVersion = talon.getFirmwareVersion( );
        error = checkTalonError(talon, baseStr + "getFirmwareVersion error");

        if (error == ErrorCode.OK)
        {
          if (fwVersion >= Falcon500.kTalonReqVersion)
          {
            talonValid = true;
            break;
          }
          else
            DataLogManager
                .log(classString + ": Talon ID " + deviceID + " " + baseStr + "Incorrect FW version - " + (fwVersion / 256.0));
        }

        Timer.delay(0.100);
      }
    }

    if (talonValid)
    {
      baseStr += "ver " + (fwVersion / 256.0) + " ";
      error = talon.configFactoryDefault( );
      if (error != ErrorCode.OK)
        DataLogManager.log(
            classString + ": Talon ID " + deviceID + " error " + error + " " + baseStr + " Message: configFactoryDefault error");
      else
        initialized = true;
    }

    if (talonValid && initialized)
      DataLogManager.log(classString + ": Talon ID " + deviceID + " error " + error + " " + baseStr + "is INITIALIZED!");
    else
      DataLogManager.log(classString + ": Talon ID " + deviceID + " error " + error + " " + baseStr + "is UNRESPONSIVE!");

    return talonValid && initialized;
  }

  public boolean talonFXInitialize(WPI_TalonFX talon, String motorName)
  {
    return talonInitialize(talon, motorName);
  }

  public boolean talonSRXInitialize(WPI_TalonSRX talon, String motorName)
  {
    return talonInitialize(talon, motorName);
  }

  public void talonFXFaultDump(WPI_TalonFX talon, String motorName)
  {
    String baseStr = ": Talon " + motorName + ": ";
    Faults faults = new Faults( );
    StickyFaults sticky = new StickyFaults( );

    // Now the Talon has been validated
    talon.getFaults(faults);
    talon.getStickyFaults(sticky);
    talon.clearStickyFaults(100);

    DataLogManager.log(classString + ": " + baseStr + "faults - " + ((faults.hasAnyFault( )) ? faults.toString( ) : "none"));
    DataLogManager.log(classString + ": " + baseStr + "sticky - " + ((sticky.hasAnyFault( )) ? sticky.toString( ) : "none"));
  }

  // CANCoder handlers

  public ErrorCode checkCANCoderError(CANCoder canCoder, String message)
  {
    ErrorCode errorCode = canCoder.getLastError( );

    if (errorCode != ErrorCode.OK)
      DataLogManager
          .log(classString + ": CANCoder ID " + canCoder.getDeviceID( ) + " error " + errorCode + " Message: " + message);

    return errorCode;
  }

  public boolean canCoderInitialize(CANCoder canCoder, String ccName)
  {
    ErrorCode error = ErrorCode.OK;
    int deviceID = 0;
    int fwVersion = 0;
    String baseStr = ccName + " ";
    boolean canCoderValid = false;
    boolean initialized = false;

    // Display Talon firmware versions
    deviceID = canCoder.getDeviceID( );
    error = checkCANCoderError(canCoder, baseStr + "getDeviceID error");

    Timer.delay(0.250);

    if (error == ErrorCode.OK)
    {
      // This can take multiple attempts before ready
      for (int i = 0; i < m_retries; i++)
      {
        fwVersion = canCoder.getFirmwareVersion( );
        error = checkCANCoderError(canCoder, baseStr + "getFirmwareVersion error");

        if (error == ErrorCode.OK)
        {
          if (fwVersion >= Falcon500.kTalonReqVersion)
          {
            canCoderValid = true;
            break;
          }
          else
            DataLogManager
                .log(classString + ": CANCoder ID " + deviceID + " " + baseStr + "Incorrect FW version - " + (fwVersion / 256.0));
        }

        Timer.delay(0.100);
      }
    }

    if (canCoderValid)
    {
      baseStr += "ver " + (fwVersion / 256.0) + " ";
      error = canCoder.configFactoryDefault( );
      if (error != ErrorCode.OK)
        DataLogManager.log(classString + ": CANCoder ID " + deviceID + " error " + error + " " + baseStr
            + " Message: configFactoryDefault error");
      else
        initialized = true;
    }

    if (canCoderValid && initialized)
      DataLogManager.log(classString + ": CANCoder ID " + deviceID + " error " + error + " " + baseStr + "is INITIALIZED!");
    else
      DataLogManager.log(classString + ": CANCoder ID " + deviceID + " error " + error + " " + baseStr + "is UNRESPONSIVE!");

    return canCoderValid && initialized;
  }

  // Pigeon IMU handlers

  private ErrorCode checkPigeonError(PigeonIMU pigeon, String message)
  {
    ErrorCode errorCode = pigeon.getLastError( );

    if (errorCode != ErrorCode.OK)
      DataLogManager.log(classString + ": Pigeon ID " + pigeon.getDeviceID( ) + " error " + errorCode + " Message: " + message);

    return errorCode;
  }

  public boolean pigeonIMUInitialize(PigeonIMU pigeon)
  {
    ErrorCode error = ErrorCode.OK;
    int deviceID = 0;
    int fwVersion = 0;
    String baseStr = new String("");
    boolean pigeonValid = false;
    boolean initialized = false;

    // Display Talon firmware versions
    deviceID = pigeon.getDeviceID( );
    error = checkPigeonError(pigeon, "getDeviceID error");

    if (error == ErrorCode.OK)
    {
      // This can take multiple attempts before ready
      for (int i = 0; i < m_retries; i++)
      {
        fwVersion = pigeon.getFirmwareVersion( );
        error = checkPigeonError(pigeon, "getFirmwareVersion error");

        if (error == ErrorCode.OK)
        {
          if (fwVersion <= Falcon500.kPigeonReqVersion)
          {
            pigeonValid = true;
            break;
          }
          else
            DataLogManager.log(
                classString + ": Pigeon ID " + deviceID + " " + baseStr + "Incorrect FW version error - " + (fwVersion / 256.0));
        }

        Timer.delay(0.100);
      }
    }

    if (pigeonValid)
    {
      baseStr += "ver " + (fwVersion / 256.0) + " ";

      error = pigeon.configFactoryDefault( );
      if (error != ErrorCode.OK)
        DataLogManager.log(
            classString + ": Pigeon ID " + deviceID + " error " + error + " " + baseStr + " Message: configFactoryDefault error");
      else
      {
        double headingDeg = pigeon.getFusedHeading( );
        error = checkPigeonError(pigeon, baseStr + "getFusedHeading error");

        boolean angleIsGood = (pigeon.getState( ) == PigeonIMU.PigeonState.Ready);
        error = checkPigeonError(pigeon, baseStr + "getState error");

        DataLogManager.log(
            classString + ": Pigeon ID " + deviceID + " " + baseStr + "fused heading: " + headingDeg + ", ready: " + angleIsGood);

        pigeon.setYaw(0.0, Constants.kLongCANTimeoutMs);
        error = checkPigeonError(pigeon, baseStr + "setYaw error");

        pigeon.setFusedHeading(0.0, Constants.kLongCANTimeoutMs);
        error = checkPigeonError(pigeon, baseStr + "setFusedHeading error");

        initialized = true;
      }
    }

    if (pigeonValid && initialized)
      DataLogManager.log(classString + ": Pigeon ID " + deviceID + " error " + error + " " + baseStr + "is INITIALIZED!");
    else
      DataLogManager.log(classString + ": Pigeon ID " + deviceID + " error " + error + " " + baseStr + "is UNRESPONSIVE!");

    return pigeonValid && initialized;
  }

  public void pigeonIMUFaultDump(PigeonIMU pigeon)
  {
    String baseStr = ": Pigeon: ";
    PigeonIMU_Faults faults = new PigeonIMU_Faults( );
    PigeonIMU_Faults sticky = new PigeonIMU_Faults( );

    // Now the Talon has been validated
    pigeon.getFaults(faults);
    pigeon.getStickyFaults(sticky);
    pigeon.clearStickyFaults(100);

    DataLogManager.log(classString + ": " + baseStr + "faults: " + ((faults.hasAnyFault( )) ? faults.toString( ) : "none"));
    DataLogManager.log(classString + ": " + baseStr + "sticky: " + ((sticky.hasAnyFault( )) ? sticky.toString( ) : "none"));
  }

  // Deprecated - use methods above

  @Deprecated
  public static void checkError(ErrorCode error, String message)
  {
    if (error != ErrorCode.OK)
      DataLogManager.log(classString + ": " + "CTRE Error code: " + error + " Message: " + message);
  }
}
