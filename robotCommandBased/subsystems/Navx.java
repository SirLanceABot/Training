package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

public class Navx extends AHRS {

    AHRS ahrs;

    Navx(AHRS navx)
    {
        ahrs = navx;

        // create the GYRO needed for this subsystem
        try {
            /***********************************************************************
             * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB. - See
             * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
             * 
             * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
             * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
             * 
             * Multiple navX-model devices on a single robot are supported.
             ************************************************************************/
            // ahrs = new AHRS(SerialPort.Port.kUSB1);
            ahrs = new AHRS(SerialPort.Port.kUSB1, SerialDataType.kProcessedData, (byte) 60); // kUSB1 - inside type A
            Timer.delay(1.0); // make sure AHRS USB communication is done before doing
            // other USB. Also give it time to settle in; a few more seconds never hurt
            ahrs.enableLogging(true);
            Shuffleboard.getTab("Gyroscope").add((Sendable) ahrs);
            System.out.println("NavX update " + ahrs.getActualUpdateRate() + " " + ahrs.getUpdateCount() + " " + ahrs.getRequestedUpdateRate());
            // ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte) 200);
            // ahrs = new AHRS(SPI.Port.kMXP);
            // ahrs = new AHRS(I2C.Port.kMXP);
            // ahrs = new AHRS(I2C.Port.kOnboard, (byte) 60);
            System.out.println("NavX update " + ahrs.getActualUpdateRate() + " " + ahrs.getUpdateCount() + " " + ahrs.getRequestedUpdateRate());
        } catch (final RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX:  " + ex.getMessage(), true);
        }

        ahrs.zeroYaw();
        // end create GYRO
    }
 
    public void displayGyro()
    {
        final float RobotHeading = ahrs.getYaw();
        SmartDashboard.putNumber("IMU_Yaw", RobotHeading/* ahrs.getYaw() */);
        SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
        SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());
        SmartDashboard.putString("Rotation2d", ahrs.getRotation2d().toString());
        SmartDashboard.putNumber("Rotation2d radians", ahrs.getRotation2d().getRadians());
        
    
        /* Display tilt-corrected, Magnetometer-based heading (requires */
        /* magnetometer calibration to be useful) */
        SmartDashboard.putBoolean("Magnetic Disturbance", ahrs.isMagneticDisturbance());
        SmartDashboard.putBoolean("Magnetometer Calibrated", ahrs.isMagnetometerCalibrated());
    
        SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());
    
        /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
        SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());
    
        /* These functions are compatible w/the WPI Gyro Class, providing a simple */
        /* path for upgrading from the Kit-of-Parts gyro to the navx MXP */
    
        SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
        SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());
    
        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
    
        SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
        SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
        SmartDashboard.putNumber("IMU_Accel_Z", ahrs.getWorldLinearAccelZ());
        SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
        SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());
    
        /* Display estimates of velocity/displacement. Note that these values are */
        /* not expected to be accurate enough for estimating robot position on a */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially */
        /* double (displacement) integration. */
    
        SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
        SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
        SmartDashboard.putNumber("Velocity_Z", ahrs.getVelocityZ());
        SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
        SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());
        SmartDashboard.putNumber("Displacement_Z", ahrs.getDisplacementZ());
    
        /* Display Raw Gyro/Accelerometer/Magnetometer Values */
        /* NOTE: These values are not normally necessary, but are made available */
        /* for advanced users. Before using this data, please consider whether */
        /* the processed data (see above) will suit your needs. */
    
        SmartDashboard.putNumber("RawGyro_X", ahrs.getRawGyroX());
        SmartDashboard.putNumber("RawGyro_Y", ahrs.getRawGyroY());
        SmartDashboard.putNumber("RawGyro_Z", ahrs.getRawGyroZ());
        SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
        SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
        SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
        SmartDashboard.putNumber("RawMag_X", ahrs.getRawMagX());
        SmartDashboard.putNumber("RawMag_Y", ahrs.getRawMagY());
        SmartDashboard.putNumber("RawMag_Z", ahrs.getRawMagZ());
        SmartDashboard.putNumber("IMU_Temp_C", ahrs.getTempC());
        SmartDashboard.putNumber("IMU_Timestamp", ahrs.getLastSensorTimestamp());
    
        SmartDashboard.putNumber("Fused Heading", ahrs.getFusedHeading());
    
        /* Omnimount Yaw Axis Information */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
        final AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
        SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
        SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());
        // out.println(AHRS.BoardAxis.kBoardAxisX + " " +
        // AHRS.BoardAxis.kBoardAxisY + " " + AHRS.BoardAxis.kBoardAxisZ +
        // yaw_axis.board_axis.getValue());
        final String kBoardAxisAlpha[] = { "BoardAxisX", "BoardAxisY", "BoardAxisZ" };
        SmartDashboard.putString("YawAxisAlpha", kBoardAxisAlpha[yaw_axis.board_axis.getValue()]);
    
        /* Sensor Board Information */
        SmartDashboard.putString("FirmwareVersion", ahrs.getFirmwareVersion());
    
        /* Quaternion Data */
        /* Quaternions are fascinating, and are the most compact representation of */
        /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
        /* from the Quaternions. If interested in motion processing, knowledge of */
        /* Quaternions is highly recommended. */
        SmartDashboard.putNumber("QuaternionW", ahrs.getQuaternionW());
        SmartDashboard.putNumber("QuaternionX", ahrs.getQuaternionX());
        SmartDashboard.putNumber("QuaternionY", ahrs.getQuaternionY());
        SmartDashboard.putNumber("QuaternionZ", ahrs.getQuaternionZ());
    
        /* Connectivity Debugging Support */
        SmartDashboard.putNumber("IMU_Byte_Count", ahrs.getByteCount());
        SmartDashboard.putNumber("IMU_Update_Count", ahrs.getUpdateCount());
        // if (ahrs->IsAltitudeValid()) // Aero only
        // {
        // SmartDashboard::PutNumber( "Barometric Pressure",
        // ahrs->GetBarometricPressure() );
        // SmartDashboard::PutNumber( "Altitude", ahrs->GetAltitude() );
        // SmartDashboard::PutNumber( "Pressure", ahrs->GetPressure() );
        // }
        // else
        // {
        // SmartDashboard::PutString( "Barometric Pressure", (llvm::StringRef)"Not
        // Available" );
        // SmartDashboard::PutString( "Altitude", (llvm::StringRef)"Not Available" );
        // SmartDashboard::PutString( "Pressure", (llvm::StringRef)"Not Available" );
        // }
    
        /* Display 6-axis Processed Angle Data */
        SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
        SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
}
}