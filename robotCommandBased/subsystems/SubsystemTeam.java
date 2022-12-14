/**
 * define method to read all inputs at once (well, sequentially) for all registered SubsystemTeam
 */
package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SubsystemTeam extends SubsystemBase
{
    private final static ArrayList<SubsystemTeam> m_subsystemArrayList = new ArrayList<SubsystemTeam>();
    
    public SubsystemTeam()
    {
        super();
        m_subsystemArrayList.add( (SubsystemTeam) this );
    }

    public void readPeriodicInputs()
    {
        DriverStation.reportError( "override readPeriodicInputs", true );
    }

    public void writePeriodicOutputs()
    {
        DriverStation.reportError( "override writePeriodicOutputs", true );
    }
    
    public static void readPeriodic()
    {
      m_subsystemArrayList.forEach( SubsystemTeam::readPeriodicInputs );
    }
  
    public static void writePeriodic()
    {
      m_subsystemArrayList.forEach( SubsystemTeam::writePeriodicOutputs );
    }
}