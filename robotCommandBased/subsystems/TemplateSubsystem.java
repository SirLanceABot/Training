package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController; // example
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TemplateSubsystem  extends Subsystem4237
{

  public TemplateSubsystem(XboxController driverController) // pass in all the stuff this class needs from above
    {
      registerPeriodicIO();
      
      periodicIO = new PeriodicIO(); // all the inputs appear here
      
      this.driverController = driverController;  // example, pass in all the stuff this class needs from above

      new DefaultCommand(); // instantiate TemplateSubsystem's (optional) default command
    }
   
  @Override
  public void readPeriodicInputs()
    {
        // populate each input variable
        periodicIO.dummy = System.currentTimeMillis(); // example
    }

  @Override
  public void writePeriodicOutputs()
  {
    //System.out.println("write outputs TemplateSubsystem");
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(periodicIO.dummy); // example
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  /**
   * define all the inputs to be read at once
   */
  private PeriodicIO periodicIO;
  private XboxController driverController; // example

  public class PeriodicIO
  {
      // INPUTS
      public long dummy; // example
      // OUTPUTS
  }
  /**
   * end define inputs
   */

  /**
   * define (optional) default command for TemplateSubsystem
   * 
   * can be done here or elsewhere in another class
   * 
   * this inner class gets variables from its outer class; is that so wrong?
   */
  class DefaultCommand extends CommandBase
  {
    // constructor
    DefaultCommand()
      {
        // example, set (optional) default command for the TemplateSubsystem
        TemplateSubsystem.this.setDefaultCommand( defaultCommand );
      }
      
  RunCommand defaultCommand = new RunCommand(
    // define the command to execute
    () ->
    {
      new WaitCommand(5.); // example
    },
    // requirement is required for a default command - maybe not needed in 2023
    TemplateSubsystem.this
    );     

  }
  /**
   * end define default command
   */
}
