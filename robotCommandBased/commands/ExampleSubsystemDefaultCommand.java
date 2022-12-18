package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExampleSubsystemDefaultCommand extends CommandBase {

  private int printCount;

  /**
   * Creates a new ExampleSubsystemDefaultCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleSubsystemDefaultCommand(ExampleSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

/**
 * example of overriding the default method that normally returns false
 */
  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    System.out.println("ExampleSubsystemDefaultCommand initialized");
    printCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    printCount++;
    if(printCount>=50)
    {
      System.out.println("ExampleSubsystemDefaultCommand printing every 50th");
      printCount = 0;
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {System.out.println("ExampleSubsystemDefaultCommand end, command interrupted = " + interrupted);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
