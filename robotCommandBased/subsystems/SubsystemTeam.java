/**
 * define method to read all inputs at once (well, sequentially) for all registered SubsystemTeam
 */
package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SubsystemTeam extends SubsystemBase
{
    // list of all registered SubsystemTeam classes
    private final static ArrayList<SubsystemTeam> m_subsystemArrayList = new ArrayList<SubsystemTeam>();
    
    public SubsystemTeam()
    {
        super(); // do the usual WPILib stuff first
        m_subsystemArrayList.add( (SubsystemTeam) this ); // register this subsystem for our periodicIO
    }

    abstract public void readPeriodicInputs(); // force others to have this implemented

    abstract public void writePeriodicOutputs(); // force others to have this implemented

    // read inputs for all registered subsystems
    public static void readPeriodic()
    {
      m_subsystemArrayList.forEach( (subsystem) -> subsystem.readPeriodicInputs() );
    }
  
    // write outputs for all registered subsystems
    public static void writePeriodic()
    {
      m_subsystemArrayList.forEach( (subsystem) -> subsystem.writePeriodicOutputs() );
    }
}

