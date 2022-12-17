/**
 * Finite State Machine for a three-speed ceiling.
 * The fan has a pull chain (Xbox button A) that switches off - high - medium - low - off.
 * The fan has an off switch (Xbox button B) that goes to off no matter the current speed.
 * The buttons only respond in teleop enabled
 */

package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.FanFSM;

public class FanFSMSubsystem  extends SubsystemTeam {
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }

  FanFSM fanFSMCommand;

  public FanFSMSubsystem(XboxController driverController) // pass in all the stuff this class needs from above
    {  
        this.driverController = driverController;
        mPeriodicIO = new PeriodicIO(); // all the inputs appear here
        fanFSMCommand = new FanFSM(this, this.driverController);
    }
   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run without regard to DISABLED or ENABLED
      fanFSMCommand.checkStateChange();
      }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  /**
   * define all the inputs to be read at once
   */
  public PeriodicIO mPeriodicIO;
  private XboxController driverController;
  
  @Override
  public void readPeriodicInputs()
    {
      // populate each input variable
      // SmartDashboard.putString(this.getName() + " read", "readPeriodicInputs");
      mPeriodicIO.AButtonPressed = driverController.getAButtonPressed();
      mPeriodicIO.BButtonPressed = driverController.getBButtonPressed();
    }

    public void writePeriodicOutputs()
    {
      // SmartDashboard.putString(this.getName() + " write", "writePeriodicOutputs");
    }
    
    
  public class PeriodicIO {
  // INPUTS
    public boolean AButtonPressed;
    public boolean BButtonPressed;
  }
  /**
   * end define inputs
   */
}

