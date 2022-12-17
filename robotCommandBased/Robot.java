package frc.robot;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SubsystemTeam;

//FIXME  comment from CD
/*
FWIW, a good way to initialize motors is to do this via a state machine,
one config call per iteration of Periodic(). You can also check to see 
if the controller has rebooted, and restart the state machine if this occurs.
From your description, you may be maxing out CAN bandwidth, triggering
errors on some of the configuration calls. Once you start hitting error
paths, odds of something unusual happening are higher.
*/

// order of execution: previous_modeExit, modeInit, modePeriodic, robotPeriodic

class Robot extends TimedRobot {
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  
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
    m_robotContainer = new RobotContainer();
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
    // System.out.println("robotPeriodic");System.out.flush();
    SubsystemTeam.readPeriodic();
    CommandScheduler.getInstance().run();
    SubsystemTeam.writePeriodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit()
  {
    // Cancels all running commands.
    DriverStation.reportWarning("Canceling all commands in Disabled", false);
    CommandScheduler.getInstance().cancelAll(); // be careful - there might be something you want to keep running
  }

  @Override
  public void disabledPeriodic() { }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(); 

    if(m_autonomousCommand != null)
    {
      System.out.println("scheduling autocommand");
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousExit() {
    System.out.println("auto exit");
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
        System.out.println("auto exit command not null");
        m_autonomousCommand.cancel();
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

