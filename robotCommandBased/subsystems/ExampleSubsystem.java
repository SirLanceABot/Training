package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ExampleCommandBuilder;
import frc.robot.commands.ExampleSubsystemDefaultCommand;

public class ExampleSubsystem extends SubsystemBase {
  
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem()
    {
      final ExampleSubsystemDefaultCommand m_defaultCommand = new ExampleSubsystemDefaultCommand(this);
      setDefaultCommand( m_defaultCommand );
      configureCommands();
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
    System.out.println("read inputs ExampleSubsystem");
  }
  
  public void writePeriodicOutputs()
{
  System.out.println("write outputs ExampleSubsystem");
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
    System.out.print("scheduled example command ID ");
    if(i == 2)
    {
      command.schedule(false); // specify interruptible (true) or not interruptible (false)
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
/*
created ExampleCommand 0
scheduled example command ID initialize for command 0
is interruptible - scheduled is true
created ExampleCommand 1
scheduled example command ID end by interrupted true for command 0
initialize for command 1
is interruptible - scheduled is true
created ExampleCommand 2
scheduled example command ID end by interrupted true for command 1
initialize for command 2
not interruptible - scheduled is true
created ExampleCommand 3
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 4
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 5
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 6
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 7
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 8
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 9
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 10
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 11
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 12
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 13
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 14
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 15
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 16
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 17
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 18
scheduled example command ID is interruptible - scheduled is false
created ExampleCommand 19
scheduled example command ID is interruptible - scheduled is false
********** Robot program startup complete **********
printed 1 for command 2
ExampleSubsystem.periodic(): 0.001980s
ExampleCommand.execute(): 0.027958s
ExampleCommand.execute(): 0.027958s
printed 2 for command 2
printed 3 for command 2
printed 4 for command 2
printed 5 for command 2
printed 6 for command 2
printed 7 for command 2
printed 8 for command 2
printed 9 for command 2
printed 10 for command 2
printed 11 for command 2
printed 12 for command 2
printed 13 for command 2
printed 14 for command 2
printed 15 for command 2
printed 16 for command 2
printed 17 for command 2
printed 18 for command 2
printed 19 for command 2
printed 20 for command 2
printed 21 for command 2
printed 22 for command 2
printed 23 for command 2
printed 24 for command 2
printed 25 for command 2
printed 26 for command 2
printed 27 for command 2
printed 28 for command 2
printed 29 for command 2
printed 30 for command 2
printed 31 for command 2
printed 32 for command 2
printed 33 for command 2
printed 34 for command 2
printed 35 for command 2
printed 36 for command 2
printed 37 for command 2
printed 38 for command 2
printed 39 for command 2
printed 40 for command 2
printed 41 for command 2
printed 42 for command 2
printed 43 for command 2
printed 44 for command 2
printed 45 for command 2
printed 46 for command 2
printed 47 for command 2
printed 48 for command 2
printed 49 for command 2
printed 50 for command 2
printed 51 for command 2
printed 52 for command 2
printed 53 for command 2
printed 54 for command 2
printed 55 for command 2
printed 56 for command 2
printed 57 for command 2
printed 58 for command 2
printed 59 for command 2
printed 60 for command 2
printed 61 for command 2
printed 62 for command 2
printed 63 for command 2
printed 64 for command 2
printed 65 for command 2
printed 66 for command 2
printed 67 for command 2
printed 68 for command 2
printed 69 for command 2
printed 70 for command 2
printed 71 for command 2
printed 72 for command 2
printed 73 for command 2
printed 74 for command 2
printed 75 for command 2
printed 76 for command 2
printed 77 for command 2
printed 78 for command 2
printed 79 for command 2
printed 80 for command 2
printed 81 for command 2
printed 82 for command 2
printed 83 for command 2
printed 84 for command 2
printed 85 for command 2
printed 86 for command 2
printed 87 for command 2
printed 88 for command 2
printed 89 for command 2
printed 90 for command 2
printed 91 for command 2
printed 92 for command 2
printed 93 for command 2
printed 94 for command 2
printed 95 for command 2
printed 96 for command 2
printed 97 for command 2
printed 98 for command 2
printed 99 for command 2
printed 100 for command 2
printed limit reached so isFinished
end by interrupted false for command 2
ExampleSubsystemDefaultCommand printing every 20th
ExampleSubsystemDefaultCommand printing every 20th
*/