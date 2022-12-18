package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.FlywheelSubsystem;

public class Autonomous1Command extends SequentialCommandGroup
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
    addRequirements(flywheelSubsystem); //all subsystems that might be used in any command in the group
    testit = new SpinupFlywheelCommand(flywheelSubsystem, 0.);
  }

  // build command from other commands and return the single big command
  public Command get()
  {
   return new SequentialCommandGroup
        (
          new SpinupFlywheelCommand( flywheelSubsystem, 300. ) .withTimeout(5.)

         ,new InstantCommand( ()->System.out.println("IC 1") )

         ,new WaitCommand(6.)
         
         ,new ParallelRaceGroup(
               testSeqGrpCommand
              ,new WaitCommand(1.))

          // same as the above ParallelRaceGroup but possibly less obvious   
          //  ,testSeqGrpCommand
          //     .raceWith(
          //   new WaitCommand(5.))

         ,new InstantCommand(() -> System.out.println("IC 2"))

         ,new PrintCommand("testing PrintCommand -  it's really identical to above print command")

         ,new WaitCommand(2.)
         
         ,new SpinupFlywheelCommand( flywheelSubsystem, 1000. ).withTimeout(6.)

         ,testit.spinAtSpeed(500.) .withTimeout(4.)
        );
  }
}

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