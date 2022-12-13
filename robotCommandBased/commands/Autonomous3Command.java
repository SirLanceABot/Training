package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Autonomous3Command extends SequentialCommandGroup
{
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }
  
  private final TestSeqGrpCommand testSeqGrpCommand = new TestSeqGrpCommand(); //all subsystems that might be used in any command in the group
 
  public Autonomous3Command(Subsystem... requiredSubsystems)
  {
    addRequirements(requiredSubsystems);
  }

  // build command from other commands and return the single big command
  public Command get()
  {
   return new SequentialCommandGroup
        (
          testSeqGrpCommand.raceWith(new WaitCommand(8.)), // give it 3 sec then move on
          testSeqGrpCommand.raceWith(new WaitCommand(5.)) // give it 3 sec then move on
        );

  }
}
