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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DriveSubsystem extends SubsystemBase {
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }

  /**
   * define default command for DriveSubsystem
   * this inner class gets variables from its outer class; is that so wrong?
   */
  class DefaultCommand extends CommandBase
  {
    RunCommand defaultCommand = new RunCommand(
            // define the command to execute
            () -> {
              setTestMotorPctVBus.accept(mPeriodicIO.driverControllerLeftX);
            },
            // requirement required for a default command - maybe not needed in 2023
            DriveSubsystem.this
            );

    DefaultCommand()
      { // once a default is set it cannot be changed to null or missing.
        // maybe a new one could be set, though, that does nothing if desired.
        DriveSubsystem.this.setDefaultCommand( defaultCommand );
      }    
  }
  /**
   * end define default command
   */

  /**
   * define all the inputs to be read at once
   */
  private PeriodicIO mPeriodicIO;
  public class PeriodicIO {
      // INPUTS
      public double PctOutput;
      public double busVoltage;
      public double voltageCompensation;
      public double velocity;
      public double driverControllerLeftX;
  }
  /**
   * end define inputs
   */
    
  XboxController driverController;
  Navx gyro; //////////// NAVX GYRO INPUT ////////////
  Supplier<Double> getVelocity;
  Runnable printSpeed;
  Consumer<Double> setTestMotorPctVBus;
  Supplier<Double> getPctOutput;
  Supplier<Double> getBusVoltage;
  Supplier<Double> getVoltageCompensation;

  public DriveSubsystem(XboxController driverController)
    {
      mPeriodicIO = new PeriodicIO();
      this.driverController = driverController; 
      createTestMotorController(configRetries); //FIXME DEFINE A CONFIG MODE TO RUN IN PERIODIC
      new DefaultCommand(); // instantiate Drive's default command

      gyro = new Navx(gyro);      
      
    }
  
  @Override
  public void periodic() {  // This method will be called once per scheduler run
    // I/O
    mPeriodicIO.PctOutput = getPctOutput.get();
    mPeriodicIO.busVoltage = getBusVoltage.get();
    mPeriodicIO.voltageCompensation = getVoltageCompensation.get();
    mPeriodicIO.velocity = getVelocity.get();
    mPeriodicIO.driverControllerLeftX = driverController.getLeftX();

    // other periodic processes
    System.out.println(mPeriodicIO.velocity + " RPM driving");
    gyro.displayGyro(); // get the GYRO values

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

    // none of these faster settings seems to work - data still slow to update; don't know why
    testMotor.setControlFramePeriodMs(10);
    errors += check(testMotor, "set control frame period", true);

    testMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); // 10 put values in Constants
    errors += check(testMotor, "set status frame 0 period", true);

    testMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); // 20
    errors += check(testMotor, "set status frame 1 period", true);

    testMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // 50
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
      SmartDashboard.putNumber("%VBus", getPctOutput.get());
      SmartDashboard.putNumber("bus voltage", getBusVoltage.get());
      SmartDashboard.updateValues();
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
  }

  public void setDriveStraightSlowly()
  {
    setTestMotorPctVBus.accept(.15);
  }

  public void setDriveStop()
  {
    setTestMotorPctVBus.accept(0.);
  }
}
