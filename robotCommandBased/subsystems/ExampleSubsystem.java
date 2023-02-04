package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ExampleCommandBuilder;

public class ExampleSubsystem extends Subsystem4237
{
  private PeriodicIO periodicIO;

  private class PeriodicIO
  {
  // INPUTS
  // OUTPUTS
  }
  /**
   * end define I/O
   */

  private int printCount;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem()
    {
      registerPeriodicIO();
      periodicIO = new PeriodicIO(); // all the inputs and outputs appear here
      configureCommands();
      printCount = 0;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void readPeriodicInputs()
  {
    SmartDashboard.putNumber(this.getName() + ".readPeriodicInputs()", ++printCount);

    // System.out.println("read inputs ExampleSubsystem");
  }
  
  public void writePeriodicOutputs()
{
    SmartDashboard.putNumber(this.getName() + ".writePeriodicOutputs()", ++printCount);
}

////////////////////////////////////////////////////////////////////////////////
///////  some TRIGGERS, BUTTONS, AND COMMANDS for this subsystem  //////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

private void configureCommands()
{
  ExampleCommandBuilder ecb = new ExampleCommandBuilder(this);

  // a bunch of commands are created and scheduled
  for(int i=0; i<20; i++)
  {
    var command = ecb.getCommand(i);
        // schedule the autonomous command (example)
  if (command != null) {
    System.out.print("scheduled example command ID " + i);
    if(i == 2)
    {
      // scduling this early - disabled so don't cancel commands in disabled
      command.withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).schedule();
      System.out.println(" not interruptible - scheduled is " + command.isScheduled());
    }
    else
    {
      command.schedule(); // default is interruptible
      System.out.println(" is interruptible - scheduled is " + command.isScheduled());
    }
  }
  }
}
}
