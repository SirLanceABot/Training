package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PeriodicIO;

/**
 * This abstract class will be extended for every subsystem on the robot. 
 * Every subsystem will automatically be added to the array list for periodic inputs and outputs.
 */
public abstract class Subsystem4237 extends SubsystemBase implements PeriodicIO
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // *** CLASS CONSTRUCTOR ***
    public Subsystem4237()
    {
        super();

        // Register this subsystem in the array list for periodic inputs and outputs.
        registerPeriodicIO();
    }

    // Abstract methods to override in subclasses
    public abstract void readPeriodicInputs();
    public abstract void writePeriodicOutputs();
}
