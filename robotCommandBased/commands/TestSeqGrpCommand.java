package frc.robot.commands;


import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.print;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestSeqGrpCommand extends SequentialCommandGroup {
    static
    {
        System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
    }
 
    public TestSeqGrpCommand()
    {
        addCommands(
            waitSeconds(1.)
            ,print("between waits")
           ,waitSeconds(2.)/*.deadlineWith(new spinupMotor(motorSubsystem, 100.))*/
        );
    }
}

// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.paths.FiveBallPartFour;
// import frc.paths.FiveBallPartOne;
// import frc.paths.FiveBallPartThree;
// import frc.paths.FiveBallPartTwo;
// import frc.paths.Spinnnn;
// import frc.robot.Robot;
// import frc.robot.drive.Drivetrain;
// import frc.robot.drive.commands.ResetOdometry;
// import frc.robot.drive.commands.TrajectoryFollower;
// import frc.robot.intake.Intake;
// import frc.robot.intake.commands.FastIntake;
// import frc.robot.intake.commands.RetractIntake;
// import frc.robot.shooter.Shooter;
// import frc.robot.shooter.commands.FlywheelController;
// import frc.robot.shooter.commands.PullTrigger;
// import frc.robot.shooter.commands.ResetEncoder;
// import frc.robot.shooter.commands.ResetHood;
// import frc.robot.shooter.commands.StopShooter;
// import frc.robot.shooter.commands.StopTrigger;
// import frc.robot.status.Status;
// import frc.robot.status.actions.ImageAction;
// import frc.robot.status.commands.ActionCommand;
// import frc.robot.status.commands.SetColor;

// public class FiveBallAuto extends SequentialCommandGroup {
//   public FiveBallAuto(Drivetrain drive, Intake intake, Shooter shooter) {
//     addCommands(
//       new ResetOdometry(drive, new Pose2d(new Translation2d(-0.7, 0), Rotation2d.fromDegrees(-90.0))),
//       new ResetEncoder(shooter),
//       new ParallelDeadlineGroup(
//         new SequentialCommandGroup(
//             new WaitCommand(0.8), // Give shooter time to spin up & hood to move
//             new PullTrigger(shooter, intake),
//             new WaitCommand(0.5)),
//         new ActionCommand(new ImageAction(Robot.fiveBallAutoImage, 0.02, ImageAction.FOREVER).brightness(0.7).oscillate()),
//         new TrajectoryFollower(drive, new FiveBallPartOne()), // Turn to point at center
//         new FlywheelController(shooter, 1810, 77.90)),
//     new ParallelDeadlineGroup(
//       new StopShooter(shooter),
//       new StopTrigger(shooter, intake),
//       new FastIntake(intake)),
//     new ParallelDeadlineGroup(
//       new WaitCommand(5.0),
//       new SequentialCommandGroup(
//         new WaitCommand(1.1), 
//         new FlywheelController(shooter, 1980, 73.25)),
//       new TrajectoryFollower(drive, new FiveBallPartTwo()),
//       new SequentialCommandGroup(
//         new WaitCommand(3.25),
//         new PullTrigger(shooter, intake)),
//       new SequentialCommandGroup(
//         new WaitCommand(4.0),
//         new RetractIntake(intake))),
//     new StopShooter(shooter),
//     new StopTrigger(shooter, intake)
//     // new ParallelDeadlineGroup(
//     //   new TrajectoryFollower(drive, new FiveBallPartThree()),
//     //   new FastIntake(intake)),
//     // new WaitCommand(0.9), // Pick up balls 4 & 5
//     // new ParallelDeadlineGroup(
//     //   new WaitCommand(4.5),
//     //   new SequentialCommandGroup(
//     //     new WaitCommand(1.75),
//     //     new FlywheelController(shooter, 1795, 77.60)),
//     //   new SequentialCommandGroup(
//     //     new WaitCommand(2.9),
//     //     new PullTrigger(shooter)),
//     //   new SequentialCommandGroup(
//     //     new WaitCommand(1.5),
//     //     new RetractIntake(intake)),
//     //   new TrajectoryFollower(drive, new FiveBallPartFour())),
//     // new StopShooter(shooter),
//     // new StopTrigger(shooter),
//     // new ResetHood(shooter),
//     // new SetColor(Status.getInstance(), Color.kBlack)
//     );
//   }
// }
/*

SequentialCommandGroup
A SequentialCommandGroup (Java, C++) runs a list of commands in sequence - the first command
 will be executed, then the second, then the third, and so on until the list finishes. The
  sequential group finishes after the last command in the sequence finishes. It is therefore
   usually important to ensure that each command in the sequence does actually finish (if a 
   given command does not finish, the next command will never start!).

ParallelCommandGroup
A ParallelCommandGroup (Java, C++) runs a set of commands concurrently - all commands will
 execute at the same time. The parallel group will end when all commands have finished.

ParallelRaceGroup
A ParallelRaceGroup (Java, C++) is much like a ParallelCommandgroup, in that it runs a set of
 commands concurrently. However, the race group ends as soon as any command in the group ends
  - all other commands are interrupted at that point.

ParallelDeadlineGroup
A ParallelDeadlineGroup (Java, C++) also runs a set of commands concurrently. However, the
 deadline group ends when a specific command (the "deadline") ends, interrupting all other 
 commands in the group that are still running at that point.
 */
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// package frc.robot.commands.commandGroups;

// import java.util.function.Supplier;

// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commands.TurnForDegrees;
// import frc.robot.lib.ShooterPosition;
// import frc.robot.subsystems.Collector;
// import frc.robot.subsystems.Drive;
// import frc.robot.subsystems.Launcher;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class FiveBallAutoGroup extends SequentialCommandGroup {
//   /** Creates a new FiveBallAutoGroup. */
//   public FiveBallAutoGroup(
//   Trajectory fiveBallAutoTrajectory1,
//   Trajectory fiveBallAutoTrajectory2,
//   Trajectory fiveBallAutoTrajectory3,
//   //Trajectory fiveBallAutoTrajectory4,
//   Supplier<Command> fiveBallRamseteCommand1, 
//   Supplier<Command> fiveBallRamseteCommand2, 
//   Supplier<Command> fiveBallRamseteCommand3, 
//   //Supplier<Command> fiveBallRamseteCommand4, 
//   Collector collector, 
//   Launcher launcher, 
//   Drive drive) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//       //new InstantCommand(() -> {
//       //   drive.setFieldTrajectory("fiveBallAuto1", fiveBallAutoTrajectory1);
//       //   drive.setFieldTrajectory("fiveBallAuto2", fiveBallAutoTrajectory2);
//       //   drive.setFieldTrajectory("fiveBallAuto3", fiveBallAutoTrajectory3);
//       // }, drive),
//       new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.LINE), launcher),
//       new InstantCommand(() -> launcher.pidOn(), launcher),
//       new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
//       new InstantCommand(() -> collector.collectIntake(), collector),
//       new InstantCommand(() -> collector.moverForward(), collector),
//       new InstantCommand(() -> collector.singulatorIntake(), collector),
//       //new InstantCommand(()-> drive.resetOdometry(fiveBallAutoTrajectory1.getInitialPose())),
//       fiveBallRamseteCommand1.get(),
//       new InstantCommand(() -> collector.moverOff(), collector),
//       new TurnForDegrees(177, drive),
//       new InstantCommand(() -> collector.feederOn(), collector),
//       new InstantCommand(() -> collector.moverForward(), collector),
//       new WaitCommand(0.5),
//       new InstantCommand(() -> collector.feederOff(), collector),
//       new TurnForDegrees(-95, drive),
//      // new InstantCommand(() -> drive.resetOdometry(fiveBallAutoTrajectory2.getInitialPose())),
//       fiveBallRamseteCommand2.get(),
//       new WaitCommand(0.5),
//       new TurnForDegrees(-155, drive),
//       new InstantCommand(()-> collector.moverOff(), collector),
//       //new InstantCommand(()-> drive.resetOdometry(fiveBallAutoTrajectory3.getInitialPose())),
//       new ParallelDeadlineGroup(
//         new SequentialCommandGroup(
//           new WaitCommand(2),
//           new InstantCommand(() -> collector.feederOn(), collector),
//           new InstantCommand(() -> collector.moverForward(), collector)
//           ),
//         fiveBallRamseteCommand3.get()
//       ),
//       new WaitCommand(2),
//       new InstantCommand(() -> launcher.pidOff(), launcher),
//       new InstantCommand(() -> collector.feederOff(), collector),
//       new InstantCommand(()-> collector.moverOff(), collector),
//       new InstantCommand(()-> collector.singulatorOff(), collector),
//       new InstantCommand(()-> collector.collectorStop(), collector)
//     );
//   }
// }