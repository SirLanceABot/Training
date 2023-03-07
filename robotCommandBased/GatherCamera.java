package frc.robot;

// This will send images back to the Dashboard or browser.
// Image will flip depending on robot heading away from or toward driver.
// * processed image (flipped or not) http://10.42.37.2:1182/stream.mjpg
// * raw image http://10.42.37.2:1181/stream.mjpg
// settings - use URL without the stream.mjpg

/*
 * USAGE:
 * 
 * Plug USB camera into roboRIO - make sure it works okay with the CANivore
 * 
 * Run this initializing code once say in RobotContainer.java
 * 
    GatherCamera gatherCamera = new GatherCamera();
    m_visionThread = new Thread(gatherCamera, "GatherCamera");
    m_visionThread.setDaemon(true);
    m_visionThread.start();
 * 
 */
import java.lang.invoke.MethodHandles;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;

public class GatherCamera implements Runnable {

private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

// *** STATIC INITIALIZATION BLOCK ***
// This block of code is run first when the class is loaded
static
{
    System.out.println("Loading: " + fullClassName);

    System.loadLibrary(Core.NATIVE_LIBRARY_NAME); //OpenCV library
}

public void run()
{
    // Get the UsbCamera from CameraServer and start it
    /*UsbCamera camera =*/ CameraServer.startAutomaticCapture();

    // Set the resolution
    // camera.setResolution(640, 480);

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo();

    // Setup a CvSource

    CvSource outputStream = CameraServer.putVideo("GatherCamera", 160, 120);

    // Mats are very memory expensive. Lets reuse this Mat.
    Mat mat = new Mat();

        
    // //FIXME: remove when gyro is used
    // SmartDashboard.putNumber("test yaw", 0.); // for testing without a gyro

    // This cannot be 'true'. The program will never exit if it is. This
    // lets the robot stop this thread when restarting robot code or
    // deploying.
    while (!Thread.interrupted()) {
        // Tell the CvSink to grab a frame from the camera and put it
        // in the source mat.  If there is an error notify the output.
        if (cvSink.grabFrame(mat) == 0) {
            // Send the output the error.
            outputStream.notifyError(cvSink.getError());
            // skip the rest of the current iteration
            continue;
        }

        // if robot not pointing downfield then flip the image left/right

        //FIXME: replace below heading with gyro get yaw
        if( Robot.yawTS.get() > 90. && Robot.yawTS.get() <= 270. )
        {
            // flip left/right
            Core.flip(mat, mat, 1);

            //FIXME: remove this description
            Imgproc.putText(mat, "toward DS", new Point(10, 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,
            new Scalar(255, 255, 255), 1);
        }
        else
        {
            //FIXME: remove this description
            Imgproc.putText(mat, "away from DS", new Point(10, 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,
            new Scalar(255, 255, 255), 1);
        }

        // Give the output stream a new image to display
        outputStream.putFrame(mat);
    }
}
}