/**
 * sample Sensor-like class that easily gets an example value
 * and displays it
 */

package frc.robot.Sensors;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PeriodicIO;

public class TemplateSensor  implements PeriodicIO
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    public class PeriodicIO
    {
        //INPUTS
        private double x;
    }

    private PeriodicIO periodicIO;

    public TemplateSensor()
    {   
        registerPeriodicIO(); // register this for PeriodicIO
        periodicIO = new PeriodicIO();
    }

    /** example getter
     * Usage periodicIO.getX()
     * @return X
     */
    public double getX()
    {
        return periodicIO.x;
    }

    @Override
    public void readPeriodicInputs() 
    {
        periodicIO.x = periodicIO.x = 3.14159;
    }

    @Override
    public void writePeriodicOutputs() 
    {
        SmartDashboard.putNumber("Pi", periodicIO.x);
    }  
}
