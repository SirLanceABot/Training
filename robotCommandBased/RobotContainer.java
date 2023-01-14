package frc.robot;

import static frc.robot.Constants.driverControllerID;
import static frc.robot.Constants.Drive.autoMinimalMoveTime;
import static frc.robot.Constants.Flywheel.*;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.Autonomous1Command;
import frc.robot.commands.Autonomous2Command;
import frc.robot.commands.Autonomous3Command;
import frc.robot.commands.Autonomous4Command;
import frc.robot.commands.Autonomous5Command;
import frc.robot.commands.ExampleSubsystemDefaultCommand;
import frc.robot.commands.SpinupFlywheelCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FanFSMSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }

//_________________________________________________________________________________
//
//____________________   USER SELECTABLE COMPONENTS  ______________________________
//_________________________________________________________________________________

// activate or not debug logging
// activate or not debug logging
// activate or not debug logging

  private boolean useDataLog         = false; // this uses space on roboRIO which runs out after some time of logging
  private boolean useShuffleBoardLog = false; // record a ShuffleBoard session then you convert playback to view

// activate or not selected subsystems
// activate or not selected subsystems
// activate or not selected subsystems

  private boolean useFullRobot       = true; // true implies all the rest are true; don't use "final" to prevent dead code messages
  private boolean useDrive           = false;
  private boolean useFlywheel        = false;
  private boolean useExample         = false;
  private boolean useFanFSM          = false;
//_________________________________________________________________________________
//_________________________________________________________________________________
//_________________________________________________________________________________

private final XboxController driverController = new XboxController(driverControllerID);
  // private final ArrayList<SubsystemTeam> m_subsystemArrayList = new ArrayList<SubsystemTeam>();

  /////////////////////////////////////////
  // SUBSYSTEMS
  /////////////////////////////////////////
  private final ExampleSubsystem exampleSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;
  private final FanFSMSubsystem fanFSMSubsystem;

  private final SendableChooser<AutoChoice> autoChooser;


  // WPILog
  StringLogEntry commandLogEntry;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  RobotContainer() {

    if(useDataLog)
    {
      // DataLog log = new DataLog("/home/lvuser", "MyUStestLog"+System.currentTimeMillis()+".wpilog");
      DataLogManager.start();
      DataLog log = DataLogManager.getLog();
      var name = new String("/Commands/"); // make a prefix tree structure for the data
      commandLogEntry = new StringLogEntry(log, name+"events", "Event");
    }

    if(!useFullRobot) DriverStation.reportWarning("NOT USING FULL ROBOT", false);
    
    exampleSubsystem  = (useFullRobot || useExample  ? new ExampleSubsystem()                  : null);
    driveSubsystem    = (useFullRobot || useDrive    ? new DriveSubsystem(driverController)    : null);
    flywheelSubsystem = (useFullRobot || useFlywheel ? new FlywheelSubsystem(driverController) : null);
    fanFSMSubsystem   = (useFullRobot || useFanFSM   ? new FanFSMSubsystem(driverController)   : null);
    
    // autonomous component
    autoChooser       = (useFullRobot                ? new SendableChooser<AutoChoice>()       : null);
  
    // clear faults, etc.
    resetRobot();
    
    // record command events for analysis
    configureSchedulerLog();

    // Configure the Driver/Operator Game Controllers buttons' bindings
    configureButtonBindings();

    // Configure the default commands for all subsystems
    configDefaultSubsystemCommands();

    // display the autonomous command selector
    configureAutoChooser();
  }

  /**
   * resetRobot to clear faults, etc.
   */
  void resetRobot()
  {
    PowerDistribution pd = new PowerDistribution();
    pd.clearStickyFaults();
    pd.close();
  }

/////////////////////////////////////////
// AUTONOMOUS COMMANDS
/////////////////////////////////////////

  public static enum AutoChoice
  {
      kAuto1, kAuto2, kAuto3, kAuto4, kAuto5, kAuto6, kAuto7, kAuto8
  }

  private void configureAutoChooser()
  {
    /*
     * warning bizarre behavior of the Chooser.
     * The selection is retained in NT through a Restart Robot Code
     * But after the restart the first periodic cycle returns the Default Value
     * and then the second periodic value (approximately, apparently) returns
     * the retained previously selected value.
     */
    //_________________________________________________________________________________

    autoChooser.addOption( "Auto 1", AutoChoice.kAuto1);        
    //_________________________________________________________________________________

    autoChooser.addOption( "Auto 2", AutoChoice.kAuto2 );
    //_________________________________________________________________________________

    autoChooser.addOption( "Spinup Flywheel to " + kAutoSpinupRPM
                 + " RPM for " + kAutoTime + " seconds", AutoChoice.kAuto3 );
    //_________________________________________________________________________________

    autoChooser.addOption("Auto 3", AutoChoice.kAuto4 );
    //_________________________________________________________________________________

    autoChooser.addOption("Auto 4", AutoChoice.kAuto5 );
    //_________________________________________________________________________________

    int count = 80; //FIXME redundant - put in Constants
    double timeout = .001;
    autoChooser.addOption("print " + count + " times in " + timeout +
         " seconds (nope-timeout doesn't do anything on an instant command)", AutoChoice.kAuto6 );
    //_________________________________________________________________________________

    autoChooser.addOption("minimal move", AutoChoice.kAuto7 );
    //_________________________________________________________________________________

    autoChooser.setDefaultOption("Auto None", AutoChoice.kAuto8 );
    //_________________________________________________________________________________

    SmartDashboard.putData("Auto choices", autoChooser);
    //_________________________________________________________________________________
  }

  /**
   * Get autonomous selection from user
   * @return user selected autonomous choice
   */
  AutoChoice getAutoChoice ()
  {
    return autoChooser.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * This turns the user choice into a Command to be scheduled
   *
   * @return the command to run in autonomous
   */
  Command getAutonomousCommand(AutoChoice autoChoice)
  {
    Command autoCommand;

    switch(autoChoice)
    {
      case kAuto1:
                  autoCommand = new Autonomous1Command(flywheelSubsystem).get();
                  break;

      case kAuto2:
                  autoCommand = new Autonomous2Command(flywheelSubsystem).get();
                  break;

      case kAuto3:
                  autoCommand =  new SpinupFlywheelCommand(flywheelSubsystem, kAutoSpinupRPM)
                    .withTimeout(kAutoTime);
                  break;

      case kAuto4:
                  autoCommand = new Autonomous3Command(driveSubsystem, flywheelSubsystem);
                  break;

      case kAuto5:
                  autoCommand = new Autonomous4Command(100, driveSubsystem, flywheelSubsystem);
                  break;

      case kAuto6:
                  int count = 80;
                  double timeout = .001; // timeout not used on InstantCommand
                  autoCommand =  new Autonomous5Command(count, timeout);
                  break;

      case kAuto7:
                  autoCommand = new RunCommand // run repeatedly
                    ( driveSubsystem::DriveStraightSlowly, driveSubsystem )
                    .withTimeout( autoMinimalMoveTime)
                    .andThen( driveSubsystem.testLambda ) // typed Runnable since no parm no return
                    .andThen( driveSubsystem::TestMethod ) // method no parm no return is implied Runnable; no requirements so 1st arg is assumed a command
                    .andThen( driveSubsystem::TestMethod, driveSubsystem ) // method no parm no return is implied Runnable; requirements so 1st arg assumed a Runnable
                    .andThen( ( )->{ System.out.println("lambda"); } ) // lambda no parm no return is implied Runnable
                    .andThen( driveSubsystem::DriveStop, driveSubsystem ) // only executed once; make RunCommand to multi-execute
                    .andThen( driveSubsystem::DriveStop, driveSubsystem ) // only executed once; make RunCommand to multi-execute
                    .andThen( driveSubsystem::DriveStop, driveSubsystem ) // only executed once; make RunCommand to multi-execute
                                                                        // or duplicate the individuals a few times to make sure it stops
                    ;
                  break;

      case kAuto8:
                  autoCommand =  new InstantCommand
                  (
                    ()->
                    {
                      var message = "No autonomous command set";
                      DriverStation.reportWarning(message, false);
                    }
                    /*no other subsystems required to display the message*/
                  );
                  break;
        
      default:
                  DriverStation.reportError("UNKNOWN AUTO COMMAND SELECTED - NONE USED", false);
                  autoCommand = null;
                  break;
    }
  
    return autoCommand;
  }
/////////////////////////////////////////
// end AUTONOMOUS COMMANDS
/////////////////////////////////////////


/////////////////////////////////////////
// Command Event Loggers
/////////////////////////////////////////
  void configureSchedulerLog()
  {
    if(useShuffleBoardLog || useDataLog)
    {
    // Set the scheduler to log events for command initialize, interrupt,
    // finish, execute
    // Log to the ShuffleBoard and the WPILib data log tool.
    // If ShuffleBoard is recording these events are added to the recording. Convert
    // recording to csv and they show nicely in Excel. 
    // If using data log tool, the recording is automatic so run that tool to retrieve and convert the log.
    //_________________________________________________________________________________

    CommandScheduler.getInstance()
        .onCommandInitialize(
            command ->
            {
              if(useDataLog) commandLogEntry.append(command.getClass() + " " + command.getName() + " initialized");
              if(useShuffleBoardLog) Shuffleboard.addEventMarker(
                  "Command initialized", command.getName(), EventImportance.kNormal);
            }
        );
    //_________________________________________________________________________________

    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command ->
            {
              if(useDataLog) commandLogEntry.append(command.getClass() + " " + command.getName() + " interrupted");
              if(useShuffleBoardLog) Shuffleboard.addEventMarker(
                    "Command interrupted", command.getName(), EventImportance.kNormal);
            }
        );
    //_________________________________________________________________________________

    CommandScheduler.getInstance()
        .onCommandFinish(
            command ->
            {
              if(useDataLog) commandLogEntry.append(command.getClass() + " " + command.getName() + " finished");
              if(useShuffleBoardLog)  Shuffleboard.addEventMarker(
                    "Command finished", command.getName(), EventImportance.kNormal);
            }
        );
    //_________________________________________________________________________________

    CommandScheduler.getInstance()
        .onCommandExecute( // this can generate a lot of events
            command ->
            {
              if(useDataLog) commandLogEntry.append(command.getClass() + " " + command.getName() + " executed");
              if(useShuffleBoardLog)  Shuffleboard.addEventMarker(
                    "Command executed", command.getName(), EventImportance.kNormal);
            }
        );
    //_________________________________________________________________________________
    }
  }

////////////////////////////////////////////////////////////////////////////////
///////    BUTTONS    //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
  /**
   * Configure teleop driver/operator controls - at least in part; others may be elsewhere
   */
  private void configureButtonBindings()
  {
//_________________________________________________________________________________

    new JoystickButton(driverController, XboxController.Button.kX.value)
      .toggleOnTrue( // 1st press starts command; 2nd press interrupts command
        new SequentialCommandGroup
        (
          new PrintCommand("toggle X button flywheel"),
          new SpinupFlywheelCommand(flywheelSubsystem, kDriverButtonFlywheelSpeed)
        ) );
//_________________________________________________________________________________
// The FanFSMSubsystem demonstrates traditional usage of getting the status of a
// button with all the other PeriodIO values and using that value in the Fan FSM.
// Those bindings are not established here in RobotContainer.
//_________________________________________________________________________________
  }

////////////////////////////////////////////////////////////////////////////////
///////    DEFAULT COMMANDS    /////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
  private void configDefaultSubsystemCommands()
  {
//_________________________________________________________________________________

    driveSubsystem.setDefaultCommand( driveSubsystem.joystickDriveCommand() );
//_________________________________________________________________________________

    exampleSubsystem.setDefaultCommand( new ExampleSubsystemDefaultCommand(exampleSubsystem) );
//_________________________________________________________________________________
  }
}

// NOTE CommandGroupBase deprecated and replaced by statics in Commands
// CommandGroupBase.clearGroupedCommands();

// If command already exists and in case a different command is requested after the first time through,
// can't reuse grouped commands without first ungrouping them.
// CommandGroupBase.clearGroupedCommand(m_autonomousCommand);
// or ungroup them all with CommandGroupBase.clearGroupedCommands
// CommandGroupBase.clearGroupedCommand(m_autonomousCommand);
//_________________________________________________________________________________--------------------------
// Instead of using the WPILib JoystickButton you can make your own class that extends Button.
// Then you make a get() method that returns the joystick value like below.
// You could then get the joystick value in the PeriodicIO if you want.
//
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.button.Button;
// /**
//  * A {@link Button} that gets its state from an {@link XboxController}.
//  */
// public class XBoxControllerButton extends Button
// {
//     private final XBoxControllerEE m_joystick;
//     private final int m_buttonNumber;
//     /**
//      * Create a joystick button for triggering commands.
//      *
//      * @param joystick The XboxController object that has that button
//      * @param kb   The button number (see {@link Button})
//      */
//     public XBoxControllerButton(XBoxControllerEE joystick, XboxController.Button kb)
//     {
//         m_joystick = joystick;
//         m_buttonNumber = kb.value;
//     }
// 	/**
//      * Gets the value of the joystick button.
//      *
//      * @return The value of the joystick button
//      */
//     public boolean get()
//     {
//         return m_joystick.getRawButton(m_buttonNumber);
//     }
// }

/////////////////////////////////////////////////////////////////////////////////////////////
// private void configureButtonBindings() {
//   Trigger magSensor = new Trigger(() -> this.innerMagazine.magSense.get());
//   Trigger turretAligned =
//       new Trigger(() -> this.vision.getTargetAligned() && this.turret.alignEnabled);
//   // Turn default lights back to 0 with start button.
//   new JoystickButton(operator, XboxController.Button.kStart.value)
//       .whenPressed(new InstantCommand(() -> leds.pattern = 0));
//   // Turn default lights to 2 with POV up (0)
//   new POVButton(operator, 0).whenPressed(new InstantCommand(() -> leds.pattern = 2));
//   // LEDs are blue when ball is loaded
//   magSensor.and(turretAligned.negate())
//       .whileActiveContinuous(new StartEndCommand(() -> leds.setColor(Color.kBlue), () -> {
//       }, leds));
//   // LEDs are green when ball is loaded and locked on
//   magSensor.and(turretAligned)
//       .whileActiveContinuous(new StartEndCommand(() -> leds.setColor(Color.kGreen), () -> {
//       }, leds));
//   // LEDs are red when limelight aligned but ball not loaded
//   magSensor.negate().and(turretAligned)
//       .whileActiveContinuous(new StartEndCommand(() -> leds.setColor(Color.kRed), () -> {
//       }, leds));
//   /* Driver Buttons */
//   // Reset Gyro on Driver Y pressed
//   new JoystickButton(driver, XboxController.Button.kY.value)
//       .whenPressed(new InstantCommand(() -> swerveDrive.zeroGyro()));
//   // Turn Off Turret For Rest of Match on Driver X Pressed
//   new JoystickButton(operator, XboxController.Button.kX.value)
//       .whenPressed(new InstantCommand(() -> turret.alignEnabled = !turret.alignEnabled));

//   /* Button Mappings for Climber Motors */
//   // Extend the Outside climber arms
//   new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
//       .whileHeld(new StartEndCommand(() -> outsideClimber.engageMotors(),
//           () -> outsideClimber.stopMotors(), outsideClimber));
//   // Retract the Outside climber arms
//   new AxisButton(driver, XboxController.Axis.kLeftTrigger.value)
//       .whileHeld(new StartEndCommand(() -> outsideClimber.retractMotors(),
//           () -> outsideClimber.stopMotors(), outsideClimber));
//   // Extend the Inside climber arms
//   new JoystickButton(driver, XboxController.Button.kRightBumper.value)
//       .whileHeld(new StartEndCommand(() -> insideClimber.engageMotors(),
//           () -> insideClimber.stopMotors(), insideClimber));
//   // Retract the Inside climber arms
//   new AxisButton(driver, XboxController.Axis.kRightTrigger.value)
//       .whileHeld(new StartEndCommand(() -> insideClimber.retractMotors(),
//           () -> insideClimber.stopMotors(), insideClimber));

//   // Inside Pneumatics Activate on drive
//   new JoystickButton(driver, XboxController.Button.kB.value)
//       .whenPressed(new OutsidePC(outsideClimber));
//   // Outside Pneumatics Activate on driver
//   new JoystickButton(driver, XboxController.Button.kA.value)
//       .whenPressed(new InsidePC(insideClimber));
//   new JoystickButton(driver, XboxController.Button.kStart.value)
//       .whenPressed(new InstantCommand(() -> insideClimber.enableClimbers())
//           .andThen(new InstantCommand(() -> outsideClimber.enableClimbers()))
//           .andThen(new InstantCommand(() -> turret.alignEnabled = false))
//           // Turns default LEDS to 1
//           .andThen(new InstantCommand(() -> leds.pattern = 1)));

//   /* Operator Buttons */

//   // Enable Shooter Tape Line setpoint right trigger
//   new AxisButton(operator, XboxController.Axis.kRightTrigger.value)
//       .whileHeld(new StartEndCommand(() -> turret.alignEnabled = true,
//           () -> turret.alignEnabled = false))
//       .whileHeld(new ShooterRPM(this.shooter, 2400 / 60))
//       .whileHeld(new FeedShooter(innerMagazine, outerMagazine, shooter, intake))
//       .whileHeld(new WheelsIn(swerveDrive));

//   // Enable Shooter Safety Location setpoint right trigger
//   new AxisButton(operator, XboxController.Axis.kLeftTrigger.value)
//       .whileHeld(new StartEndCommand(() -> turret.alignEnabled = true,
//           () -> turret.alignEnabled = false))
//       .whileHeld(new ShooterRPM(this.shooter, 3250 / 60)) // 15 ft
//       .whileHeld(new FeedShooter(innerMagazine, outerMagazine, shooter, intake))
//       .whileHeld(new WheelsIn(swerveDrive));

//   // Enable Shooter Magazine Combo While Operator A Button Held
//   new JoystickButton(operator, XboxController.Button.kA.value)
//       .whileHeld(new StartEndCommand(() -> turret.alignEnabled = true,
//           () -> turret.alignEnabled = false))
//       .whileHeld(new ShooterRPM(this.shooter, this.vision))
//       .whileHeld(new FeedShooter(innerMagazine, outerMagazine, shooter, intake))
//       .whileHeld(new WheelsIn(swerveDrive));

//   // Deploy Intake and Run Magazine While Operator B Held
//   new JoystickButton(operator, XboxController.Button.kB.value)
//       .whileHeld(new StartEndCommand(() -> {
//           intake.intakeDeploy();
//           outerMagazine.magazineUp();
//       }, () -> {
//           intake.intakeRetract();
//           outerMagazine.magazineStop();
//       }, intake, outerMagazine).alongWith(new InnerMagIntake(innerMagazine)));
//   // Run hopper down with POV down (180))
//   new POVButton(operator, 180).whileHeld(new StartEndCommand(() -> {
//       innerMagazine.magazineDown();
//       outerMagazine.magazineDown();
//   }, () -> {
//       innerMagazine.magazineStop();
//       outerMagazine.magazineStop();
//   }));
//   // Right Turret Move While Operator Right Bumper Held
//   new JoystickButton(operator, XboxController.Button.kRightBumper.value).whileHeld(
//       new StartEndCommand(() -> turret.turretRight(), () -> turret.turretStop(), turret));

//   // Left Turret Move While Operator Left Bumper Held
//   new JoystickButton(operator, XboxController.Button.kLeftBumper.value).whileHeld(
//       new StartEndCommand(() -> turret.turretLeft(), () -> turret.turretStop(), turret));

//   // Spit ball command
//   new JoystickButton(operator, XboxController.Button.kY.value)
//       .whileHeld(new SequentialCommandGroup(new InstantCommand(() -> shooter.spinShooter()),
//           new WaitCommand(.2), new InstantCommand(() -> {
//               innerMagazine.magazineUp();
//               outerMagazine.magazineUp();
//           })))
//       .whenReleased(new InstantCommand(() -> {
//           shooter.stopShooter();
//           innerMagazine.magazineStop();
//           outerMagazine.magazineStop();
//       }));
