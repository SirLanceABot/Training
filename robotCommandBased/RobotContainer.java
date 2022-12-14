package frc.robot;

import java.lang.invoke.MethodHandles;

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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autonomous1Command;
import frc.robot.commands.Autonomous2Command;
import frc.robot.commands.Autonomous3Command;
import frc.robot.commands.Autonomous4Command;
import frc.robot.commands.Autonomous5Command;
import frc.robot.commands.SpinupFlywheelCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FanFSMSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.SubsystemTeamManagerSubsystem;
import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }
 
  boolean yes = true;
  boolean no = false;

// activate or not selected subsystems
// activate or not selected subsystems
// activate or not selected subsystems

  private final boolean useFullRobot        = yes; // includes using autonomous chooser commands
  private final boolean useDrive            = no;
  private final boolean useFlywheel         = no;
  private final boolean useExample          = no;
  private final boolean useFanFSM           = no;

  private final XboxController driverController = new XboxController(driverControllerID);
  
  /////////////////////////////////////////
  // SUBSYSTEMS
  /////////////////////////////////////////
  private final SubsystemTeamManagerSubsystem mSubsystemManager = new SubsystemTeamManagerSubsystem(); // put first
  
  private final ExampleSubsystem exampleSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;
  private final FanFSMSubsystem fanFSMSubsystem;

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  // include the useAutonomous for all the subsystems required by an autonomous command
  public RobotContainer() {

    if(!useFullRobot) DriverStation.reportWarning("NOT USING FULL ROBOT", false);
    
    exampleSubsystem  = (useFullRobot || useExample  ? instantiateSubsystem(new ExampleSubsystem())                  : null);
    driveSubsystem    = (useFullRobot || useDrive    ? instantiateSubsystem(new DriveSubsystem(driverController))    : null);
    flywheelSubsystem = (useFullRobot || useFlywheel ? instantiateSubsystem(new FlywheelSubsystem(driverController)) : null);
    fanFSMSubsystem   = (useFullRobot || useFanFSM   ? instantiateSubsystem(new FanFSMSubsystem(driverController))   : null);

    autoChooser       = (useFullRobot                ?                      new SendableChooser<Command>()           : null);

    // display all the auto choices on the SmartDashboard
    if(autoChooser != null)
    {
      configureAutoChooser();
    }

    // clear faults, etc.
    resetRobot();
    
    // record command events for analysis
    configureSchedulerLog();

    // Configure the Driver/Operator Game Controllers buttons' bindings
    configureButtonBindings();
  }

  /**
   * Add instantiation of a subsystem to the mass periodic I/O scheme
   * @param <T> class type of subsystem
   * @param instantiated subsystem object
   * @return object then can be assigned to a variable
   */
  <T> T instantiateSubsystem(T subsystem)
  { 
    mSubsystemManager.addSubsystem(subsystem);
    return subsystem;
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
  /**
   * Method to configure the chooser for the Autonomous Selection
   */
  private void configureAutoChooser()
  {
      // CommandGroupBase.clearGroupedCommands();
      // autonomous commands and the subsystems each command requires
//---------------------------------------------------------------------------------
      autoChooser.addOption("Auto 1",
            new Autonomous1Command(flywheelSubsystem).get());
//---------------------------------------------------------------------------------
      autoChooser.addOption("Auto 2",
            new Autonomous2Command(exampleSubsystem).get());
//---------------------------------------------------------------------------------
      autoChooser.addOption("Spinup Flywheel to " + Constants.Flywheel.kAutoSPinRPM
                          + " RPM for " + Constants.Flywheel.kAutoTime + " seconds",
            new SpinupFlywheelCommand(flywheelSubsystem, Constants.Flywheel.kAutoSPinRPM)
            .withTimeout(Constants.Flywheel.kAutoTime));
//---------------------------------------------------------------------------------
      autoChooser.addOption("Auto 3",
            new Autonomous3Command(driveSubsystem, flywheelSubsystem));
//---------------------------------------------------------------------------------
      autoChooser.addOption("Auto 4",
            new Autonomous4Command(10000).withTimeout(2.5));
//---------------------------------------------------------------------------------
      int count = 2000;
      double timeout = .001;
      autoChooser.addOption("print " + count + " times in " + timeout + " seconds",
            new Autonomous5Command(count, timeout));
//---------------------------------------------------------------------------------
      double time = 14.;
      autoChooser.addOption("drive straight slowly " + time + " seconds",
            new RunCommand
                (
                  driveSubsystem::setDriveStraightSlowly, driveSubsystem
                )
            .andThen(new WaitCommand(time))
            .andThen(driveSubsystem::setDriveStop, driveSubsystem)
            );
//---------------------------------------------------------------------------------
      autoChooser.setDefaultOption("Auto None - ERROR",
            new InstantCommand
              (
                ()->
                {
                  var message = "No autonomous command set";
                  DriverStation.reportWarning(message, true);
                }
                /*no other subsystems required to display the message*/
              )
            );
//---------------------------------------------------------------------------------
      SmartDashboard.putData("Auto choices", autoChooser);
//---------------------------------------------------------------------------------
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser == null ? null : autoChooser.getSelected();
  }
  /////////////////////////////////////////
  // end AUTONOMOUS COMMANDS
  /////////////////////////////////////////

  void configureSchedulerLog()
  {
    // Set the scheduler to log Shuffleboard events for command initialize, interrupt,
    // finish, execute
    // If ShuffkeBoard is recording these events are added to the recording. Convert
    // recording to csv and they show nicely in Excel. 
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command ->
                Shuffleboard.addEventMarker(
                    "Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command ->
                Shuffleboard.addEventMarker(
                    "Command interrupted", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command ->
                Shuffleboard.addEventMarker(
                    "Command finished", command.getName(), EventImportance.kNormal));
    
    CommandScheduler.getInstance()
        .onCommandExecute( // this can generate a lot of events
            command ->
                Shuffleboard.addEventMarker(
                    "Command executed", command.getName(), EventImportance.kNormal));

  }

  ////////////////////////////////////////////////////////////////////////////////
  ///////    BUTTONS    //////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  /**
   * Configure teleop driver/operator controls - at least in part; others may be elsewhere
   */
  private void configureButtonBindings()
  {
//---------------------------------------------------------------------------------
    double speed = 500.; // establish the shooter flywheel start/stop button at speed

    new JoystickButton(driverController, XboxController.Button.kX.value)
      .toggleWhenActive( // 1st press starts command; 2nd press interrupts command
        new SequentialCommandGroup(
          new InstantCommand( ()-> System.out.println("toggle X button flywheel")),
          new SpinupFlywheelCommand(flywheelSubsystem, speed) ) );
//---------------------------------------------------------------------------------
// The FanFSMSubsystem and FanFSMCommand demonstrate traditional usage of getting
// the status of a button with all the other PeriodIO values and using that value
// in the Fan FSM. Those bindings are not established here in RobotContainer.
//---------------------------------------------------------------------------------
  }
}

// If command already exists and in case a different command is requested after the first time through,
// can't reuse grouped commands without first ungrouping them.
// CommandGroupBase.clearGroupedCommand(m_autonomousCommand);
// or ungroup them all with CommandGroupBase.clearGroupedCommands
// CommandGroupBase.clearGroupedCommand(m_autonomousCommand);
//-----------------------------------------------------------------------------------------------------------
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
