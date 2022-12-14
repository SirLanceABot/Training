package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class FlywheelSubsystem extends SubsystemTeam {
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }

  private int printCount;

  public FlywheelSubsystem(XboxController driverController)
  {
    System.out.println("construct FlywheelSubsystem");
    createFlywheelMotorController(ParameterSetAttemptCount);
    mPeriodicIO = new PeriodicIO(); // all the inputs appear here
    this.driverController = driverController; // example, pass in all the stuff this class needs from above
    printCount = 0;
  }

  @Override
  public void readPeriodicInputs()
  {
      // populate each input variable
      mPeriodicIO.velocity = getFlywheelSpeed.get();
  }

  public void writePeriodicOutputs()
  {
    if(++printCount >= 20)
    {
      System.out.println("flywheel " + mPeriodicIO.velocity + " RPM");
      printCount = 0;
    }
  }
  
   @Override
  public void periodic() {
  // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
  }

  /**
  * define all the inputs to be read at once
  */
  private PeriodicIO mPeriodicIO;
  public XboxController driverController;

  public class PeriodicIO {
    // INPUTS
    public double velocity;
  }
  /**
  * end define inputs
  */

  /** Sample PIDF controller in one CAN TalonFX motor controller using Integrated Sensor
   * 
   * Assumes a single rotational direction such as shooter flywheel
   * 
   * SET THE PARAMETERS AS YOU WISH
   * MOST PARAMETERS ARE MOST EASILY CHANGED USING THE PHOENIX TUNER
   * 
   * IN TELEOP MODE
   * --------------
   * All the PIDF constants and filter times are built into the code and are okay for no load
   * on a the Falcon 500 motor.
   * The intention is they can be changed in the Phoenix Tuner to tune a real device.
   * 
   * The setpoint speed is entered on the SmartDashboard as "velocity set (native units)".
   * It is the only input to the program.
   * Press TAB to have the input value sent (ENTER works but then the robot is disabled).
   * 
   * IN AUTO MODE
   * ------------
   * Programs runs fully automatically at several speeds to display the calculated kF.
   * Voltage compensation is included in the calculation as selected by the user.
   */

  final int flywheelMotorPort = 0;
  final double voltageCompensation = 10.; // if using voltageCompensation, kF should be determined with it on or kFcompensation = kFbattery * battery/voltageCompensation
  final double neutralDeadband = 0.001;
  final int pidIdx = 0; // Talon primary closed loop control (or none)
  final boolean invert = false;
  final int filterWindow = 1; // ms
  final SensorVelocityMeasPeriod filterPeriod = SensorVelocityMeasPeriod.Period_5Ms; // 10ms and 20ms had less fluctuation
  final int sampleTime = 15; // ms
  final double kP = 0.1; // works for the entire velocity range
  final double kI = 0.; // if used no effect then suddenly bad
  final double kD = 0.; // if used no effect then suddenly bad
  // final double kF = 0.046; // good around 8000 nu and okay for the entire velocity range with a little more error creeping in (~12.3v battery)
  final double kF = 0.0555; // okay for the entire velocity range with a little more error creeping in (10v compensation)
  final double integralZone = 0.; // no limit
  final double maxIntegralAccumulator = 0.; // no limit

  // TalonFX magic numbers
  double nativeToRPM = 10. * 60. / 2048.; // 10 .1sec/sec   60 secs/min   rev/2048 encoder ticks for Integrated Sensor
  double PctVBusToThrottle = 1023.; // 1023 talon throttle unit / 100%VBus

  // TalonFX flywheelMotorFollower;
  TalonFX flywheelMotor;
  private static final int TIMEOUT_MS = 50; // milliseconds TalonFX command timeout limit
  Runnable printSpeed;
  public Consumer<Double> setFlywheelSpeed;
  public Supplier<Double> getFlywheelSpeed;
  public Supplier<Double> getSpeedError;
  public Consumer<Double> setFlywheelPctVBus;
  public Supplier<Double> getPctOutput;
  public Supplier<Double> getBusVoltage;
  public Supplier<Boolean> isVoltageCompensationEnabled;

  double speed = 0.; //initial speed to run and display on SmartDashboard
  // It seemed that sometimes a previous value from the SmartDashboard is used (race condition?).
  // This code tries hard to prevent but not sure it's perfect or what the issue was.
  int ParameterSetAttemptCount = 5; // retry flywheel config if error

  /**
   * create the flywheel motor controller
   * 
   * @param attemptLimit configuration number of times if any errors (always run at least once)
   */
  public void createFlywheelMotorController(int attemptLimit)
  {
    flywheelMotor = new TalonFX(flywheelMotorPort);

    int setAttemptNumber = 0;
    
    // loop if not completed okay until retry limit
    while (! configFlywheelMotorController(voltageCompensation, neutralDeadband,
                                           pidIdx, invert, filterWindow, filterPeriod, sampleTime,
                                           kP, kI, kD, kF, integralZone, maxIntegralAccumulator) )
      {
        setAttemptNumber++;
        if (setAttemptNumber >= attemptLimit)
        {
          DriverStation.reportError("[Talon] failed to initialize flywheel motor controller on CAN id " + flywheelMotorPort, false);
          System.out.println("[Talon] failed to initialize flywheel motor controller on CAN id " + flywheelMotorPort);
          break;
        }
      }
  }

  /** Configure the Talon motor controller
   * 
   * @param voltageCompensation limit motor input - 0. disables voltage compensation
   * @param neutralDeadband 0.001 to 0.25
   * @param pidIdx PID index 0 is either 0 or none
   * @param invert (motor reversed)
   * @param filterWindow ms
   * @param filterPeriod ms
   * @param sampleTime ms
   * @param kP
   * @param kI
   * @param kD
   * @param kF
   * @param integralZone integral zone (in native units) If the (absolute) closed-loop error is outside of this zone, integral accumulator is automatically cleared. This ensures than integral wind up events will stop after the sensor gets far enough from its target.
   * @param maxIntegralAccumulator Max integral accumulator (in native units)
   */
  boolean configFlywheelMotorController(double voltageCompensation, double neutralDeadband, int pidIdx,  boolean invert,
                      int filterWindow, SensorVelocityMeasPeriod filterPeriod, int sampleTime,
                      double kP, double kI, double kD, double kF, double integralZone, double maxIntegralAccumulator)
  {
      int errors = 0; // count TalonFX method errors

      flywheelMotor.clearStickyFaults(TIMEOUT_MS);
      errors += check(flywheelMotor, "clear faults", true);

      flywheelMotor.configFactoryDefault(TIMEOUT_MS);
      errors += check(flywheelMotor, "set default", true);

      // flywheelMotorFollower = new TalonFX(1);
      // System.out.println("[Talon] clear faults " + flywheelMotorFollower.clearStickyFaults(TIMEOUT_MS));
      // System.out.println("[Talon] set default " + flywheelMotorFollower.configFactoryDefault(TIMEOUT_MS));
      // flywheelMotorFollower.follow(flywheelMotor);
  
      flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, sampleTime, TIMEOUT_MS);
      errors += check(flywheelMotor, "set status 2", true);
      flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, sampleTime, TIMEOUT_MS); // PID error
      errors += check(flywheelMotor, "set status 13", true);
      
      //System.out.println("[Talon] set status 10 " + flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, sampleTime, TIMEOUT_MS)); // this may or may not be useful

      flywheelMotor.setInverted(invert);
      errors += check(flywheelMotor, "set inverted", true);

      flywheelMotor.setNeutralMode(NeutralMode.Coast);
      errors += check(flywheelMotor, "set neutral mode", true);

      flywheelMotor.setSelectedSensorPosition(0, pidIdx, TIMEOUT_MS); // start at 0 position just for fun; not needed to tune velocity
      errors += check(flywheelMotor, "set sensor position", true);
     
      // get the factory defaults for some setting as defined by this Java API - may be different than Phoenix Tuner
			TalonFXConfiguration configs = new TalonFXConfiguration();

      // change the ones that we need to

      // voltage compensation seems potentially useful but not for sure to enable tuning at realistic voltage and for reproducibility
      // kF has to be increased by the amount of voltage reduction in the compensation. Check kP, too.
      configs.voltageCompSaturation = voltageCompensation;

      // one direction only assumed and forced - these work for both inverted or not
      configs.peakOutputReverse = 0.;
      configs.peakOutputForward = 1.;

      configs.neutralDeadband = neutralDeadband;

      configs.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

      configs.velocityMeasurementWindow = filterWindow;
      configs.velocityMeasurementPeriod = filterPeriod;
      configs.slot0.kP = kP;
      configs.slot0.kI = kI;
      configs.slot0.kD = kD;
      configs.slot0.kF = kF;
      configs.slot0.integralZone = integralZone;
      configs.slot0.maxIntegralAccumulator = maxIntegralAccumulator;

      flywheelMotor.configAllSettings(configs, TIMEOUT_MS); // send the new config back
      errors += check(flywheelMotor, "set configs", true);

      flywheelMotor.getAllConfigs(configs, TIMEOUT_MS); // read them back
      errors += check(flywheelMotor, "get configs", true);

      System.out.println("[Talon] configuration:\n" + configs); // print them

      if(voltageCompensation != 0.)
      {
        flywheelMotor.enableVoltageCompensation(true);
        errors += check(flywheelMotor, "set enable compensation error", true);
      }

      System.out.println("[Talon] compensation " + flywheelMotor.isVoltageCompensationEnabled());
      errors += check(flywheelMotor, "compensation error", true);

      System.out.println("[Talon] " + errors + " errors from config methods"); //TODO return the count or retry if not 0

      //
      // methods for others to access the TalonFX motor controller
      //
      setFlywheelPctVBus = (speed) -> 
      {
        flywheelMotor.set(TalonFXControlMode.PercentOutput, speed);
        check(flywheelMotor, "set %VBus error", false);
      };

      setFlywheelSpeed = (speed) -> 
      {
        flywheelMotor.set(TalonFXControlMode.Velocity, speed);
        check(flywheelMotor, "set speed error", false);
      };

      getFlywheelSpeed = () ->
      {
        var speed = flywheelMotor.getSelectedSensorVelocity(pidIdx);
        check(flywheelMotor, "get sensor error", false);
        return speed;
      };

      getSpeedError = () ->
      {
        var loopError = flywheelMotor.getClosedLoopError(pidIdx);
        check(flywheelMotor, "get velocity_error error", false);
        return loopError;
      };

      getPctOutput = () ->
      {
        var pctOutput = flywheelMotor.getMotorOutputPercent();
        check(flywheelMotor, "get %VBus error", false);
        return pctOutput;
      };

      getBusVoltage = () ->
      {
        var volts = flywheelMotor.getBusVoltage();
        check(flywheelMotor, "get voltage error", false);
        return volts;
      };

      isVoltageCompensationEnabled = () ->
      {
        var enabled = flywheelMotor.isVoltageCompensationEnabled();
        check(flywheelMotor, "auto check comp error", false);
        return enabled;
      };
      
      // method to display stuff
      printSpeed = () ->
      {
        SmartDashboard.putNumber("velocity measured (native units)", getFlywheelSpeed.get());
        SmartDashboard.putNumber("velocity measured (RPM)", getFlywheelSpeed.get() * nativeToRPM);
        SmartDashboard.putNumber("error (native units)", getSpeedError.get());
        SmartDashboard.putNumber("error (RPM)", getSpeedError.get() * nativeToRPM);
        SmartDashboard.putNumber("kF tentative", PctVBusToThrottle *
            (isVoltageCompensationEnabled.get() ? getBusVoltage.get() / voltageCompensation : 1.) * // voltage compensation correction factor
             getPctOutput.get() / getFlywheelSpeed.get() );
        SmartDashboard.putNumber("%VBus", getPctOutput.get());
        SmartDashboard.putNumber("bus voltage", getBusVoltage.get());
        SmartDashboard.updateValues();
      };

      return errors == 0;
    }

  /** Check the TalonFX function for an error and print a message
   * 
   * @param TalonFX
   * @param message to print
   * @param printAll flag to print all (true) or just errors (false)
   * @return 1 for error and 0 for no error
   */
  public static int check(TalonFX motorController, String message, boolean printAll)
  {
    if(printAll) // assume if printing then a little delay is okay such as for configuring
    {
      Timer.delay(0.021); // try for a different iterative period
    }  
    var rc = motorController.getLastError();
    if(rc != ErrorCode.OK || printAll)
    {
      System.out.println("[Talon] " + message + " " + rc);
    }
    return rc == ErrorCode.OK ? 0 : 1;
  }
}
