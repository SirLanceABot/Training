package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import static frc.robot.Constants.Drive.*;

public class DriveSubsystem extends Subsystem4237
{
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }

  /**
   * define all the inputs to be read at once
   */
  public PeriodicIO periodicIO  = new PeriodicIO();
  public class PeriodicIO {
      // INPUTS
      public double PctOutput;
      public double busVoltage;
      public double voltageCompensation;
      public double velocity;
      public double driverControllerLeftX;
      public double encoder;
      public double accelX;
      public double accelY;
      public double accelZ;
  }
  /**
   * end define inputs
   */
    
  XboxController driverController;
  Accelerometer accelerometer;
  Supplier<Double> getVelocity;
  Runnable printSpeed;
  Consumer<Double> setTestMotorPctVBus;
  Supplier<Double> getPctOutput;
  Supplier<Double> getBusVoltage;
  Supplier<Double> getVoltageCompensation;

  public DriveSubsystem(XboxController driverController, Accelerometer accelerometer)
    {
      registerPeriodicIO();
      this.driverController = driverController;
      this.accelerometer = accelerometer; 
      createTestMotorController(configRetries); //TODO DEFINE A CONFIG MODE TO RUN IN PERIODIC

      // gyro = new Navx(gyro);      
      
    }

  @Override
  public void readPeriodicInputs()
    {
      SmartDashboard.putString(this.getName() + " read", "readPeriodicInputs");
      periodicIO.PctOutput = getPctOutput.get();
      periodicIO.busVoltage = getBusVoltage.get();
      periodicIO.voltageCompensation = getVoltageCompensation.get();
      periodicIO.encoder = getVelocity.get();
      periodicIO.driverControllerLeftX = driverController.getLeftX();
      periodicIO.accelX = accelerometer.getX();
      periodicIO.accelY = accelerometer.getY();
      periodicIO.accelZ = accelerometer.getZ();
    }

  public void writePeriodicOutputs()
  {
    SmartDashboard.putNumber(this.getName() + " velocity RPM", periodicIO.encoder);
    printSpeed.run();
    // gyro.displayGyro(); // get the GYRO values
    SmartDashboard.putNumber("Tilt X-Z", tiltXZ() );
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    // IF CONFIG MODE DO ONE CONFIG STATEMENT PER CYCLE
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * create the TestMotor motor controller
   * 
   * @param attemptLimit configuration number of times if any errors (always run at least once)
   */
  public void createTestMotorController(int attemptLimit)
  {
    CANSparkMax testMotor = new CANSparkMax(testMotorPort, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    RelativeEncoder encoder = testMotor.getEncoder();

    getVelocity = () ->
    {
      var velocity = encoder.getVelocity(); // apparently no way to check last error
      return velocity;
    };

    int setAttemptNumber = 0;
    
    // loop if not completed okay until retry limit
    while (! configTestMotorController( testMotor ) )
      {
        setAttemptNumber++;
        if (setAttemptNumber >= attemptLimit)
        {
          DriverStation.reportError("[Spark] failed to initialize TestMotor motor controller on CAN id " + testMotorPort, false);
          System.out.println("[Spark] failed to initialize TestMotor motor controller on CAN id " + testMotorPort);
          break;
        }
      }
  }

  /** Configure the Spark motor controller
   * 
   * @param motor controller to be configured
   */
  boolean configTestMotorController(CANSparkMax testMotor)
  {
    // note that many settings are not remembered through power off/on cycle unless burnFlash()
    // is used to save them. Seems unnecessary since we expect go through this config every
    // time power is turned on.
    
    int errors = 0; // count CANSparkMax method errors

    //Initialize Neo on a SparkMax
    testMotor.restoreFactoryDefaults();
    errors += check(testMotor, "factory defaults", true);

    testMotor.setIdleMode(IdleMode.kCoast);
    errors += check(testMotor, "set idle mode", true);

    // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
    // default values shown here not what's in the docs
    // none of these faster settings seems to work - data still slow to update; don't know why
    testMotor.setControlFramePeriodMs(10);
    errors += check(testMotor, "set control frame period", true);

    testMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, statusFrame0Periodms); // 10 put values in Constants
    errors += check(testMotor, "set status frame 0 period", true);

    testMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, statusFrame1Periodms); // 20
    errors += check(testMotor, "set status frame 1 period", true);

    testMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, statusFrame2Periodms); // 50
    errors += check(testMotor, "set status frame 2 period", true);

    // testMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 5); // not used?
    // errors += check(testMotor, "set status frame 3 period", true);

    testMotor.enableVoltageCompensation(VoltageCompensation);
    errors += check(testMotor, "set voltage compensation", true);
    
    testMotor.set(0);
    errors += check(testMotor, "set speed 0", true);

    System.out.println("[Spark] " + errors + " errors from config methods");

    //
    // methods for others to access the Spark motor controller
    //
    setTestMotorPctVBus = (speed) -> 
    {
      // System.out.println("drive speed " + speed);
      testMotor.set(speed);
      check(testMotor, "set %VBus error", false);
    };

    getPctOutput = () ->
    {
      var pctOutput = testMotor.getAppliedOutput();
      check(testMotor, "get %VBus error", false);
      return pctOutput;
    };

    getBusVoltage = () ->
    {
      var volts = testMotor.getBusVoltage();
      check(testMotor, "get voltage error", false);
      return volts;
    };

    getVoltageCompensation = () ->
    {
      var nominalVoltage = testMotor.getVoltageCompensationNominalVoltage();
      check(testMotor, "nominal voltage", false);
      return nominalVoltage;
    };
    
    // method to display stuff
    printSpeed = () ->
    {
      SmartDashboard.putNumber("%VBus", periodicIO.PctOutput);
      SmartDashboard.putNumber("bus voltage", periodicIO.busVoltage);
    };
    return errors == 0;
  }

  /** Check the Spark function for an error and print a message
   * 
   * @param Spark
   * @param message to print
   * @param printAll flag to print all (true) or just errors (false)
   * @return 1 for error and 0 for no error
   */
  public static int check(CANSparkMax motorController, String message, boolean printAll)
  {
    if(printAll) // assume if printing then a little delay is okay such as for configuring
    {
      Timer.delay(0.021); // try for a different iterative period
    }
    var rc = motorController.getLastError();
    if(rc != REVLibError.kOk || printAll)
    {
      System.out.println("[Spark] " + message + " " + rc);
    }
    // could get faults and getStickyfaults here
    return rc == REVLibError.kOk ? 0 : 1;
  }// You can also check to see if the controller has rebooted and refresh configs that may have been lost

  public void DriveStraightSlowly()
  {
    System.out.println("set slow motor speed");
    setTestMotorPctVBus.accept(DriveStraightSlowlySpeed);
  }

  public void DriveStop()
  {
    System.out.println("set motor off");
    setTestMotorPctVBus.accept(0.);
  }

  public void TestMethod()
  {
    System.out.println("TestMethod");
  }

  public Runnable testLambda = () -> {System.out.println("testLambda");};

  public Command joystickDriveCommand()
      { 
        return
          new RunCommand(
            // define the command to execute
            () -> {
              if(!DriverStation.isAutonomous()) // ignore joystick during auto
                setTestMotorPctVBus.accept(periodicIO.driverControllerLeftX);
                SmartDashboard.putNumber("DriverControllerLeftX", periodicIO.driverControllerLeftX);
            },
            // requirement required for a default command - maybe not needed in 2023
            DriveSubsystem.this
            );
      }

  /**
   * roboRIO tilt in degrees
   * @return angle degrees
   */
  public double tiltXZ()
  {
    // var angleXY = Math.atan2(mPeriodicIO.accelX, mPeriodicIO.accelY);
    var angleXZ = Math.atan2(periodicIO.accelX, periodicIO.accelZ);
    // var angleYZ = Math.atan2(mPeriodicIO.accelY, mPeriodicIO.accelZ);
    return angleXZ*360./(2.*Math.PI);
  }
}
