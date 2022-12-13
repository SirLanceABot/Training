package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController; // example
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TemplateSubsystem  extends SubsystemBase {

  public TemplateSubsystem(XboxController driverController) // pass in all the stuff this class needs from above
    {
      mPeriodicIO = new PeriodicIO(); // all the inputs appear here
      
      this.driverController = driverController;  // example, pass in all the stuff this class needs from above

      new DefaultCommand(); // instantiate TemplateSubsystem's (optional) default command
    }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(mPeriodicIO.dummy); // example
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  /**
   * define all the inputs to be read at once
   */
  private PeriodicIO mPeriodicIO;
  private XboxController driverController; // example

  public class PeriodicIO {
      // INPUTS
      public long dummy; // example
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
