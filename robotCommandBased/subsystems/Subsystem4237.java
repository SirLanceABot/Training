/**
 * Define methods to read all inputs at once (well, sequentially) and
 * similarly write all outputs at once for all registered Subsystem4237
 */
package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Subsystem4237 extends SubsystemBase
{
    // list of all registered SubsystemTeam classes
    private final static ArrayList<Subsystem4237> subsystemArrayList = new ArrayList<Subsystem4237>();
    
    public Subsystem4237()
    {
        super(); // do the usual WPILib stuff first
        subsystemArrayList.add( (Subsystem4237) this ); // register this subsystem for our periodicIO
    }

    abstract public void readPeriodicInputs(); // force others to have this implemented

    abstract public void writePeriodicOutputs(); // force others to have this implemented

    // read inputs for all registered subsystems
    public static void readPeriodic()
    {
      subsystemArrayList.forEach( (subsystem) -> subsystem.readPeriodicInputs() );
    }
  
    // write outputs for all registered subsystems
    public static void writePeriodic()
    {
      subsystemArrayList.forEach( (subsystem) -> subsystem.writePeriodicOutputs() );
    }

}

