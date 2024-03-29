package frc.robot;

import java.lang.invoke.MethodHandles;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.AutoChoice;
import frc.robot.Sensors.LL;
import frc.robot.Sensors.PVW;

// order of execution: previous_modeExit, modeInit, modePeriodic, robotPeriodic

class Robot extends TimedRobot {
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }

  private Command autonomousCommand;
  private AutoChoice autoChoicePrevious = null;
  private RobotContainer robotContainer;
  
  Thread m_visionThread;
  GatherCamera gatherCamera;

  PVW pvw;
  LL ll;
  
  public static AtomicReference<Double> yawTS = new AtomicReference<Double>(180.);

  Robot()
  {
    LiveWindow.disableAllTelemetry(); // don't waste time on stuff we don't need
  }
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // gatherCamera = new GatherCamera();
    // m_visionThread = new Thread(gatherCamera, "GatherCamera");
    // m_visionThread.setDaemon(true);
    // m_visionThread.start();
  
    //FIXME: remove when gyro is used
    SmartDashboard.putNumber("test yaw", 180.); // for testing without a gyro
    
    // pvw = new PVW();

    // ll = new LL();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
    // this is run after the other Periodic methods
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
  @Override
  public void robotPeriodic() 
  { 
    // uptime output
    // 940.4729565700001 940.47
    // 940.4841530020001 940.48
    // try {
    //   System.out.println((System.nanoTime()*1.e-9) + " " + (new Scanner(new FileInputStream("/proc/uptime")).next()));
    // } catch (FileNotFoundException e) {
    //   e.printStackTrace();
    // } 
    // nanoTime appears to be roboRIO uptime - that's not generally applicable to different computers, though.
    // uptime file could be a Linux standard
    // nanoTime takes a long time to fetch the time
    // fetching time from a file stream must be horrendously long (note that 2 resources aren't closed here, either, and it needs the try/catch)
    // if you can use a differential time, then use the efficient System.currentTimeMillis()

    PeriodicIO.readInputs();         // 1st
    CommandScheduler.getInstance().run(); // 2nd
    PeriodicIO.writeOutputs();        // 3rd
    
    //FIXME: replace below heading with gyro get yaw
    var heading = SmartDashboard.getNumber("test yaw", 0.); // testing without a gyro
    yawTS.set(heading);

    // pvw.PVacquire();

    // ll.LLacquire();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit()
  {
    ///////no don't do this autoChoicePrevious = null; // force refreshing the auto command every time robot disabled from another mode
    // Cancels all running commands.
    // DriverStation.reportWarning("Canceling all scheduled commands in Disabled", false);
    // CommandScheduler.getInstance().cancelAll(); // be careful - there might be something you want to keep running
  }

  @Override
  public void disabledPeriodic()
  {
    // test auto that is similar but not identical to how the real robot code will work
    // get the auto choice
    Optional<AutoChoice> autoChoice = robotContainer.getAutoChoice();
    autoChoice.ifPresent( selection ->
    {
      if (selection != autoChoicePrevious)
      { // if it has changed then get the new command for the new selection
        autonomousCommand = robotContainer.getAutonomousCommand(selection);
        DriverStation.reportWarning( "selected autonomous command " + selection, false );
        autoChoicePrevious = selection; // reset the previous choice to the new choice
      }
    }
    );
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    System.out.println("autonomousInit");
    if(autonomousCommand != null)
    {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousExit() {
    System.out.println("auto exit");
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
        autonomousCommand.cancel();
    }
  }
  
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() { }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

