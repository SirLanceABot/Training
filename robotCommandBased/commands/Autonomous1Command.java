package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

public class Autonomous1Command
{
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }
  
  private final TestSeqGrpCommand testSeqGrpCommand = new TestSeqGrpCommand();
  private final FlywheelSubsystem flywheelSubsystem;
  private final SpinupFlywheelCommand testit;

  public Autonomous1Command(FlywheelSubsystem flywheelSubsystem)
  {
    this.flywheelSubsystem = flywheelSubsystem;
    testit = new SpinupFlywheelCommand(flywheelSubsystem, 0.);
  }

  // build command from other commands and return the single big command
  public CommandBase get()
  {
    var alt = true;
    
    if(alt)
    return new SpinupFlywheelWPILibPIDcommand(3500., flywheelSubsystem);
 
    else
    return sequence
        (
          new SpinupFlywheelCommand( flywheelSubsystem, 300. ) .withTimeout(5.)

         ,runOnce( ()->System.out.println("IC 1") ) // print() is better

         ,waitSeconds(6.)
         
         ,parallel(
               testSeqGrpCommand
              ,waitSeconds(1.)) // runs a minimum of seconds

         ,print("IC 2")

         ,waitSeconds(2.)
         
         ,new SpinupFlywheelCommand( flywheelSubsystem, 1000. ) .withTimeout(6.)

         ,testit.spinAtSpeed(500.) .withTimeout(4.) // sets speed and returns the command object
        );
  }
}

// But lets say you want a command that runs as long as a button is pressed and do an action when its stopped
// <button>.whileTrue(<commandWithFinishCriteria>)
//         .whenFalse(<finalizationCommand>);
// You could also do this:
// button.whileTrue(Commands.run(runningAction).finallyDo(onCancelAction)));
// (add an until in there if you want the command to be able to finish itself)


// public class RobotContainer {
//   // The driver's controller
//   private final XboxController m_driverController =
//       new XboxController(OIConstants.kDriverControllerPort);

//   // A few commands that do nothing, but will demonstrate the scheduler functionality
//   private final CommandBase m_instantCommand1 = new InstantCommand();
//   private final CommandBase m_instantCommand2 = new InstantCommand();
//   private final CommandBase m_waitCommand = new WaitCommand(5);

//   /** The container for the robot. Contains subsystems, OI devices, and commands. */
//   public RobotContainer() {
//     // Set names of commands
//     m_instantCommand1.setName("Instant Command 1");
//     m_instantCommand2.setName("Instant Command 2");
//     m_waitCommand.setName("Wait 5 Seconds Command");
//   }

//   /**
//    * Use this method to define your button->command mappings. Buttons can be created by
//    * instantiating a {@link GenericHID} or one of its subclasses ({@link
//    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
//    * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
//    */
//   private void configureButtonBindings() {
//     // Run instant command 1 when the 'A' button is pressed
//     new JoystickButton(m_driverController, Button.kA.value).whenPressed(m_instantCommand1);
//     // Run instant command 2 when the 'X' button is pressed
//     new JoystickButton(m_driverController, Button.kX.value).whenPressed(m_instantCommand2);
//     // Run instant command 3 when the 'Y' button is held; release early to interrupt
//     new JoystickButton(m_driverController, Button.kY.value).whenHeld(m_waitCommand);
//   }

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand() {
//     return new InstantCommand();
//   }
// }

// addCommands(
//                 new InstantCommand(() -> Infeed.getInstance().setInfeedDown()),
//                 new WaitCommand(0.25),
//                 new InstantCommand(() -> Infeed.getInstance().forceRunInfeed()),
//                 new InstantCommand(() -> Shooter.getInstance().setShooterIndex(16.5, true)),
//                 util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_AcquireFirstCargo())
//                         .alongWith(new WaitCommand(1.5)
//                                 .andThen(new InstantCommand(() -> Shooter.getInstance().runShooterMotors()))),
//                 new RotateDrivetrainToAngle(Rotation2d.fromDegrees(37.5)).withTimeout(1.0),
//                 new InstantCommand(() -> Conveyor.getInstance().runConveyorMotor(VBusConstants.kConveyAll)),
//                 new WaitCommand(1.1),
//                 new InstantCommand(() -> Conveyor.getInstance().stopConveyorMotor()),
//                 new ResetDefaultIndex(),
//                 new InstantCommand(() -> Shooter.getInstance().stop()),
//                 util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_AcquireSecondCargo())
//                         .alongWith(new InstantCommand(() -> Shooter.getInstance().runShooterMotors())),
//                 new InstantCommand(() -> Conveyor.getInstance().runConveyorMotor(VBusConstants.kConveyAll)),
//                 new WaitCommand(1.1),
//                 new InstantCommand(() -> Conveyor.getInstance().stopConveyorMotor()),
//                 new InstantCommand(() -> Shooter.getInstance().stop()),
//                 util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_AcquireLoadingZoneCargo()),
//                 new WaitCommand(1.2),
//                 util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_ReturnToShoot())
//                         .alongWith(new WaitCommand(1.5)
//                                 .andThen(new InstantCommand(() -> Shooter.getInstance().runShooterMotors()))),
//                 new InstantCommand(() -> Conveyor.getInstance().runConveyorMotor(VBusConstants.kConveyAll)),
//                 new WaitCommand(1.1),
//                 new InstantCommand(() -> Conveyor.getInstance().stopConveyorMotor()),
//                 new WaitCommand(0.25),
//                 new InstantCommand(() -> Shooter.getInstance().stop()),
//                 new InstantCommand(() -> Vision.getInstance().setInfeedCamera()));



// /**
//  * Feed Shooter with mag motors
//  */
// public class FeedShooter extends SequentialCommandGroup {
//   OuterMagazine outerMagazine;
//   InnerMagazine innerMagazine;
//   Intake intake;

//   /**
//    * Feed Shooter with mag motors
//    *
//    * @param innerMagazine Inner Magazine subsystem
//    * @param outerMagazine Outer Magazine Subsystem
//    * @param shooter Shooter Subsystem
//    */
//   public FeedShooter(InnerMagazine innerMagazine, OuterMagazine outerMagazine, Shooter shooter,
//       Intake intake) {
//       addRequirements(innerMagazine, outerMagazine);
//       this.innerMagazine = innerMagazine;
//       this.outerMagazine = outerMagazine;
//       this.intake = intake;

//       SequentialCommandGroup part1 =
//           new SequentialCommandGroup(new PrintCommand("Shooter is being weird"),
//               new WaitUntilCommand(() -> shooter.getSetpoint() > 0 && shooter.atSetpoint()),
//               new WaitCommand(.2),
//               (new WaitUntilCommand(() -> !innerMagazine.magSense.get()).withTimeout(.5)
//                   .andThen(new WaitCommand(.2)))
//                       .deadlineWith(new MagazineRPM(shooter, innerMagazine)),
//               new InnerMagIntake(innerMagazine)
//                   .alongWith(new InstantCommand(() -> outerMagazine.magazineUp(.6))));
//       SequentialCommandGroup part2 = new SequentialCommandGroup(
//           new WaitUntilCommand(() -> shooter.getSetpoint() > 0 && shooter.atSetpoint()),
//           new WaitCommand(.2), new MagazineRPM(shooter, innerMagazine));

//       ParallelDeadlineGroup intakeWhileShooting = new SequentialCommandGroup(part1, part2)
//           .deadlineWith(new InstantCommand(() -> intake.intakeDeploy()));
//       addCommands(intakeWhileShooting);
//   }

//   @Override
//   public void end(boolean interrupted) {
//       this.innerMagazine.disable();
//       this.outerMagazine.magazineStop();
//       this.intake.intakeRetract();
//   }
// }


// public class AutonomousDistance extends SequentialCommandGroup {
//   /**
//    * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
//    * turn around and drive back.
//    *
//    * @param drivetrain The drivetrain subsystem on which this command will run
//    */
//   public AutonomousDistance(Drivetrain drivetrain) {
//     addCommands(
//         new DriveDistance(-0.5, 10, drivetrain),
//         new TurnDegrees(-0.5, 180, drivetrain),
//         new DriveDistance(-0.5, 10, drivetrain),
//         new TurnDegrees(0.5, 180, drivetrain));
//   }
// }
///////////////////////////////////////
// new AutoShoot(m_shooter, m_tower, ShooterConstants.kTarmacVelocity + 75, ShooterConstants.kShooterGains, Value.kForward)
// .withTimeout(1.2)
// .raceWith(new RunCommand(m_intake::fullRunIntake, m_intake))

//                 util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_AcquireFirstCargo())
//                 .alongWith(new WaitCommand(1.5)
//                 .andThen(new InstantCommand(() -> Shooter.getInstance().runShooterMotors()))),

///////////////////////////////////////////////
// public class TestAll extends SequentialCommandGroup {
//   private Climber climber = Climber.getInstance();
//   // private Conveyor conveyor = Conveyor.getInstance();
//   private Infeed infeed = Infeed.getInstance();
//   private Limelight limelight = Limelight.getInstance();
//   // private Shooter shooter = Shooter.getInstance();

//   private RunShooterMotors runShooter = new RunShooterMotors();
//   private SetShortShot sh = new SetShortShot();
//   private SetLongShot lo = new SetLongShot();

//   private WaitCommand wait = new WaitCommand(0.5);

//   private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

//   /** Creates a new TestAll. */
//   public TestAll() {
//       // Add your commands in the addCommands() call, e.g.
//       // addCommands(new FooCommand(), new BarCommand());
//       addCommands(
//           new InstantCommand(() -> limelight.setLedMode(1)),
//           new InstantCommand(() -> climber.setCoast()),
//           new WaitCommand(5.0), // 5 secs to pull the rope
//           new InstantCommand(() -> climber.setBrake()),
//           wait,
//           new InstantCommand(() -> climber.resetEncoders()),
//           new WaitUntilCommand(() -> !compressor.enabled()),
//           new InstantCommand(() -> infeed.setInfeedDown()),
//           wait,
//           new WaitCommand(2.0).deadlineWith(new RunInfeedSingulatorMotors()),
//           new RunConveyorTwoBall(),
//           parallel(
//               runShooter,
//               sequence(
//                   wait,
//                   sh,
//                   wait,
//                   sh,
//                   wait,
//                   sh,
//                   wait,
//                   lo,
//                   new WaitCommand(1.0),
//                   lo,
//                   wait,
//                   lo,
//                   wait
//               )
//           ),
//           new InstantCommand(() -> runShooter.cancel()),
//           new MoveArm(0.8, 130, false),
//           wait,
//           new ToggleGrippy(),
//           new WaitCommand(2.0),
//           new ToggleGrippy(),
//           new WaitCommand(1.0),
//           new ToggleTippy(),
//           wait,
//           new ToggleTippy(),
//           wait,
//           new MoveArm(-0.8, -130, false)
//       );
//   }
// }

// ////////////////////////////////////
// joystickButtons1[1].toggleWhenPressed(
//         new ConditionalCommand(
//             new ParallelCommandGroup(
//                 new InstantCommand(() -> collector.disableCollector(), collector),
//                 new InstantCommand(() -> storage.disableStorage(), storage),
//                 new InstantCommand(() -> flywheel.stopFlywheel(), flywheel)),
//             new SequentialCommandGroup(
//                 new InstantCommand(() -> collector.enableCollector(), collector),
//                 new SortStorageCommand(storage),
//                 new InstantCommand(() -> collector.disableCollector(), collector),
//                 new SetFlywheelVelocityCommand(flywheel, FlywheelConstants.WALL_SHOT_VELOCITY)),
//             collector::isEnabled));

// ///////////////////////////////////


// public StartEndCommand clawdownCommand = new StartEndCommand(
//     () -> arm.runarm(0.2),
//     () -> arm.stoparm(),
//   arm);

// //////////////////////////////////////

/*

prensing

Yes, I get it, at least partly. But what I missed in my description is that, after the first few obvious commands like 'drive' and 'run intake', all the interesting commands involve 2-3 subsystems and, even more challenging, have data values which need to go from one subcommand to another during the command.

So, for a simple example, a shooting command will first run the vision system to get a target lock, then feed the (conditioned?) angle to the drivetrain to turn to the target, and the distance to the shooter to set the speed. I really don’t see a clean way to move that data around from one command to another while running, apart from stashing it in a subsystem (which feels ugly), or having the subsequent commands re-fetch/compute values (which might be in different state!). Is there a clean example which does this kind of stateful progression?
Oblarg

Is there a clean example which does this kind of stateful progression?

Yes and no.

As you say, the Command-based composition model doesn’t explicitly support data-passing from one command to another. There have been some ideas banged-about for explicitly supporting this within the context of the decorator composition model - it’s tempting to imagine turning the Command lifecycle callbacks into argumented methods with a State variable, and propagating that state to subsequent commands through some defined addition to the Command API.

Unfortunately, trying to implement such a thing rapidly runs into limitations of the Java type system (in particular it has a very hard time representing the type of our supposed State variable). There’s unlikely to ever be a strict solution in this sense, unless we move to a language with a much more powerful type system (C++ may be able to do its own thing here, but the template hackery would probably be awful).

A more feasible solution is to declare command state as method locals inside of a factory method that returns a command constructed via inline syntax, with the inline functions capturing the state variables. In C++ this works just fine, but in Java it runs problems with the 'effectively final' limitation on lambda captures. This can be circumvented by placing state primitives in wrapper classes (arrays are a common hack, though any mutable data structure will do), or by writing accessor methods for them and capturing calls to those instead. At this point we’re not really saving on verbosity for a single command definition anymore, but we may still be reducing code duplication by staying within a composition model - and if a factory class provides factory methods for multiple related command types, some of the boilerplate can be shared.

Unfortunately, Java is rigid and verbose. You have to pick between the rigid verbosity of its subclassing rules, or the rigid verbosity of its functional APIs.

As a final note, I want to object slightly to this characterization:

all the interesting commands involve 2-3 subsystems and, even more challenging, have data values which need to go from one subcommand to another during the command.

It’s true that stateful commands are challenging. I think this is a good reason to try as hard as possible to avoid stateful commands except where absolutely necessary. A lot of 'interesting' commands don’t really need transient state to work well, and the apparent need for it can often be satisfied by improving the state management of the rest of the robot (e.g. so that it is robust/consistent when repeatedly queried during a single scheduler iteration).

Sometimes you do actually need a stateful command; and sometimes such stateful commands really don’t benefit much from being decomposed. But a little bit of mindfulness can make it so that this situation is an edge-case rather than the norm.
*/

// Re: stateful inline commands, here’s an example of a stateful turn-to-angle command as a subsystem factory method:

// public Command turnToAngle(double targetDegrees) {
//     // Create a controller for the inline command to capture
//     PIDController controller = new PIDController(Constants.kTurnToAngleP, 0, 0);
//     // We can do whatever configuration we want on the created state before returning from the factory
//     controller.setPositionTolerance(Constants.kTurnToAngleTolerance);

//     // Try to turn at a rate proportional to the heading error until we're at the setpoint, then stop
//     return run(() -> arcadeDrive(0,-controller.calculate(gyro.getHeading(), targetDegrees)))
//         .until(controller::atSetpoint)
//         .andThen(runOnce(() -> arcadeDrive(0, 0)));
// }

/////////////////////////////////////////////
// Ryan_Blue
// Team 1018AM | WPILib Dev
// 1d
// and the one idea at a time,

// The problem with this is that they’re not separate ideas.

// Take the following code:

// runOnce(()->elevator.setPosition(5), elevator).andThen(()->{}).until(elevator::atSetpoint);
// It’s pretty clear, even without knowledge of the command based system, what this is supposed to do: set the elevator position, then idle until the elevator is at the setpoint.

// The corresponding class code would be like this:

// public class SetElevatorPosition extends CommandBase {
//   private final Elevator m_elevator;
//   private final double m_position;

//   public SetElevatorPosition(Elevator elevator, double position) {
//     m_elevator = elevator;
//     m_position = position;
//     addRequirements(m_elevator);
//   }

//   @Override
//   public void initialize() {
//     m_elevator.setPosition(m_position);
//   }

//   @Override
//   public boolean isFinished() {
//     return m_elevator.atSetpoint();
//   }
// }
// There is just so much more boilerplate fluff which gets in the way of understanding what the command actually does. The in-line version makes it much more declarative and readable.

// Oblarg
// Robot apartments! invisible suburbs! skeleton treasuries!

// Ryan_Blue
// 1d
// You can shorten the inline syntax further by the calling Elevator.run factory instead (it’s public!).

// Bmongar
// 5013 Codenator / Electrical, CSA

// Ryan_Blue
// 1d
// When the pieces are as simple as that I agree, as the complexity grows i think there is a time to go first class object. I am not an absolutist except is my dislike of absolutism.

// And I wouldn’t have put the m_elevator.setPosition(m_position);in initialize either.

// mdurrani834

// runOnce(()->elevator.setPosition(5), elevator).andThen(()->{}).until(elevator::atSetpoint);

// While this is very clean, it’s not exactly that readable for a new student. What is the andThen(() → {}) doing? Why is there not a andThen() after the until() telling the motor to hold or stop? This is easiest when I can just read it like a sentence. 'Set the elevator position to 5, then ???, until the elevator is at its setpoint'. The second code block makes it a lot clearer and defined for a new student. I’m sure you could write your inline command clearer, but I maintain that as it grows, its understandability (if that’s a word) gets worse at a faster rate than a traditional class.

// From an educational standpoint, it’s so much easier for me to have students write these subclassed commands, understand the different parts completely, then teach them the whole inline command stuff because it makes more sense that way. Rather than struggle with understanding a command and this new syntax, they can take things one at a time. As commands get more and more complex, it becomes harder and harder to understand an inline command and it’s just plain easier to make it into a subclassed one for a student who isn’t an expert in in lining commands. For students that don’t have prior Java experience, it takes a lot of time to get them up and running with even the basics. My answer may be different if modern Java curricula included lambda expressions in their teachings, but the reality is that they aren’t.

// I think at the end of the day, it comes down to priorities and what end of the pedagogy spectrum you lie on.

// prensing
// FRC 2877 LigerBots Mentor
// 1d
// There is just so much more boilerplate fluff which gets in the way of understanding what the command actually does.

// Honestly I very much disagree. Yes there are about 20 lines but it is all very readable. The start and end conditions are explicitly there and obvious, and there is no extraneous 'idle' clause. Unless you are very immersed in Java (which I and probably most students are not) '() → {}' is very non-obvious.

// Ryan_Blue

// mdurrani834
// Looking at it again, this particular example would probably be better written as
// runOnce(()->elevator.setPosition(5), elevator).andThen(waitUntil(elevator::atSetpoint));

// Same behavior, just moves the waiting into a wait command rather than an until decorator.

// Amicus1

// Ryan_Blue

// elevator.setPositionC(5).andThen(waitUntil(elevator::atSetpoint));

// Even shorter, migrating the command creation to a factory in the subsystem.

// ngreen
// Doing that, you’d probably move the waiting and atsetpoint into the command factory, because I don’t really see a use case for setting the position without trying to complete it.

// ngreen

// Amicus1

// Doing that, you’d probably move the waiting and atsetpoint into the command factory, because I don’t really see a use case for setting the position without trying to complete it.

// Ryan_Blue

// ngreen
// Yep, this whole thing would be good as a factory. I included the elevator variable to give context.

