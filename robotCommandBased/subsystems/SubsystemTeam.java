/**
 * define method to read all inputs at once (well, sequentially) for all registered SubsystemTeam
 */
package frc.robot.subsystems;

public interface SubsystemTeam {
    public void readPeriodicInputs();
    public void writePeriodicOutputs();
}