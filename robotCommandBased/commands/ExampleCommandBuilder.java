package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ExampleSubsystem;

public class ExampleCommandBuilder
{
    static
    {
        System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
    }
    
    // subsystems that may be used
    private final ExampleSubsystem exampleSubsystem;
 
    /**
     * @param an instance of ExampleSubsystem
     */
    public ExampleCommandBuilder(ExampleSubsystem exampleSubsystem)
    {
        this.exampleSubsystem = exampleSubsystem;
    }

    /**
     * @param an instance of ExampleSubsystem
     * @param commandID
     * @return
     */
    public Command getCommand(int commandID)
    {
        return new ExampleCommand(this.exampleSubsystem, commandID, true);
    }
}
