package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemTeamManagerSubsystem extends SubsystemBase {
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }

  public static SubsystemTeamManagerSubsystem mInstance = null;

  private List<SubsystemTeam> mAllSubsystems = new ArrayList<SubsystemTeam>();
  
  public SubsystemTeamManagerSubsystem() { }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mAllSubsystems.forEach( SubsystemTeam::readPeriodicInputs );
    mAllSubsystems.forEach( SubsystemTeam::writePeriodicOutputs );
    mAllSubsystems.forEach( (SubsystemTeam subsystem) -> {SmartDashboard.putData((SubsystemBase)subsystem);} );
  }

  public List<SubsystemTeam> getSubsystems() {
      return mAllSubsystems;
  }

  public void addSubsystem(Object subsystem)
  {
    mAllSubsystems.add((SubsystemTeam)subsystem);
  }
}
