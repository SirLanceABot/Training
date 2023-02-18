package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FlywheelSubsystem;

public class Autonomous2Command extends CommandBase
{
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }
  
  private final TestSeqGrpCommand testSeqGrpCommand = new TestSeqGrpCommand();
  private final FlywheelSubsystem flywheelSubsystem;
  private final SpinupFlywheelCommand testit;

  public Autonomous2Command(FlywheelSubsystem flywheelSubsystem)
  {
     this.flywheelSubsystem = flywheelSubsystem;

  // I don't think this actually adds requirements if this class isn't run as a command. What are the references?
  // Since there are no Command methods the default isFinished() is false and thus runs forever.

  //   addRequirements(flywheelSubsystem); //all subsystems that might be used in any command in the group

    testit = new SpinupFlywheelCommand(flywheelSubsystem, 0.);
  }

  // build command from other commands and return the single big command
  public CommandBase get()
  {
   return sequence
        (
          new SpinupFlywheelCommand( flywheelSubsystem, 300. ) .withTimeout(10.)
         ,runOnce(()->System.out.println("IC 1")) // print is better
         ,testSeqGrpCommand.raceWith(new WaitCommand(5.))
         ,print("IC 2")
         ,new SpinupFlywheelCommand( flywheelSubsystem, 1000. ) .withTimeout(8.)
         ,testit.spinAtSpeed(500.) .withTimeout(4.)
        );
  }
}