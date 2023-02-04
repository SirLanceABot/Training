package frc.robot;

import java.util.ArrayList;

public interface PeriodicIO
{
    // *** CLASS & INSTANCE VARIABLES ***

    // list of all registered SubsystemTeam classes
    public final static ArrayList<PeriodicIO> allPeriodicIO = new ArrayList<PeriodicIO>();

    // Abstract methods to override in subclasses
    public abstract void readPeriodicInputs();
    public abstract void writePeriodicOutputs();

    /**
     * Register subsystem as using PeriodicIO.
     * Subsystems are on the honor system to register here.
     * 
     */
    public default void registerPeriodicIO()
    {
        allPeriodicIO.add(this);
    }

    /** read inputs for all registered subsystems
     * 
     */
    public static void readInputs()
    {
        for(PeriodicIO periodicIO : allPeriodicIO)
            periodicIO.readPeriodicInputs();
    }

    /** write outputs for all registered subsystems
     * 
     */
    public static void writeOutputs()
    {
        for(PeriodicIO periodicIO : allPeriodicIO)
            periodicIO.writePeriodicOutputs();
    }
}

/*
 example usage of PeriodicIO

  public class DriveSubsystem extends SubsystemBase implements PeriodicIO

  /**
   * define all the inputs and outputs to be processed at once
   *_/
  public PeriodicIO periodicIO  = new PeriodicIO();
  public class PeriodicIO {
      // INPUTS
      public double PctOutput;
      // OUTPUTS
  }

  @Override
  public void readPeriodicInputs()
    {
      periodicIO.PctOutput = getPctOutput.get(); // motor controller internal fractional voltage
    }

  public void writePeriodicOutputs()
  {
    SmartDashboard.putNumber("Read back motor %", periodicIO.PctOutput);
  }
*/
  