package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Autonomous4Command extends CommandBase
{
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }
 
  int count;
  int printLimit;

  public Autonomous4Command(int printLimit, Subsystem... requiredSubsystems)
  {
    this.printLimit = printLimit;

    addRequirements(requiredSubsystems);
  }

  @Override
  public void initialize()
  {
    count = 0;
  }

  @Override
  public void execute()
  {
    System.out.println(" auto 4 command print count " + count++);
  }

  @Override
  public void end(boolean interrupted) { }

  @Override
  public boolean isFinished()
  {
    if(count >= printLimit) return true;
    else return false;
  }

}
