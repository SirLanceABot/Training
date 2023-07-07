// https://docs.opencv.org/4.8.0/da/d13/tutorial_aruco_calibration.html

// almost complete conversion of the openCV sample program to calibrate a camera
// using the ChArUco Board

// C:\Users\RKT\frc\FRC2023\opencv_contrib-4.x\modules\aruco\samples\calibrate_camera_charuco.cpp
// or calibrate_camera.cpp OpenCV 4.8.0 (and up I would hope)

// works on Windows 10 using OpenCV version 4.8.0

// For the camera server based on https://github.com/wpilibsuite/StandaloneAppSamples/tree/main/Java
// and JavaCvSink.java from WPILib ShuffleBoard

// in a terminal window compile and run the program
// first SAVE the files because there is no auto save in the gradlew.bat
// .\gradlew.bat shadowJar
// java -jar build\libs\TestApplication-winx64.jar

// add to build.gradle dependencies:
// implementation wpilibTools.deps.wpilibJava("cameraserver")
// implementation fileTree(dir: 'C:\\opencv\\build\\java', include: ['*.jar'])

// the dll is in the path

// can also:
// create libs dir in project root
// implementation fileTree(dir: 'libs', include: ['*.jar']) // jars in libs dir
// put a jar file in libs
// make sure dll is in some path or put in project folder

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CharucoBoard;
import org.opencv.objdetect.CharucoDetector;
import org.opencv.objdetect.CharucoParameters;
import org.opencv.objdetect.DetectorParameters;
import org.opencv.objdetect.Dictionary;
import org.opencv.objdetect.Objdetect;
import org.opencv.objdetect.RefineParameters;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;

public class Program {

  /// USER CONFIGURATION
        static final boolean writeBoard = false; // make a ChArUco Board image in a jpg to print
        // print the board then measure it carefully; put those measurements below and rerun to calibrate
        static final float squareLength = /*0.015f;*/ .018536f;
        static final float markerLength = /*0.012f;*/ .01486f;
        static final int camId = 1; // Checks for the specified camera and uses it if present. 0 internal, 1 external if there is a 0 internal (sometimes)
        static final int cameraW = 320;
        static final int cameraH = 240;
        static final int cameraFPS = 121;
  /// end USER CONFIGURATION

        static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME); // Load the native OpenCV library
        // System.loadLibrary("opencv_videoio_ffmpeg480_64");
      }

    public static void main(String[] args) throws IOException {

        NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
        WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
        WPIMathJNI.Helper.setExtractOnStaticLoad(false);
        CameraServerJNI.Helper.setExtractOnStaticLoad(false);
        CombinedRuntimeLoader.loadLibraries(Program.class, "wpiutiljni", "wpimathjni", "ntcorejni", "cscorejnicvstatic");
        // var inst = NetworkTableInstance.getDefault(); // not using NT in this program

        // int printCount = 0;

        /// Charco Board
        // print the board then measure it carefully and put those measurements below
        final Dictionary dictionary = Objdetect.getPredefinedDictionary(Objdetect.DICT_5X5_1000);
        final int squaresX = 11;
        final int squaresY = 8;
        final CharucoBoard board = new CharucoBoard(new Size(squaresX, squaresY), squareLength, markerLength, dictionary);
        final Mat boardImage = new Mat();
        final Size boardImageSize = new Size(1280, 720);
        board.generateImage(boardImageSize, boardImage);

        if(writeBoard)
        {
          // write ChArUco Board to print and use for calibration
          final MatOfInt writeBoardParams = new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, 100); // pair-wise; param1, value1, ...
          final String file = "ChArUcoBoard.jpg";
          Imgcodecs.imwrite(
            file,
            boardImage,
            writeBoardParams);
          // HighGui.imshow("ChArUcoBoard", boardImage);
          // HighGui.waitKey();
          System.exit(0);
        }
        ///

        /// detector parameters
        final DetectorParameters detectParams = new DetectorParameters();
        final RefineParameters refineParams = new RefineParameters();
        final CharucoParameters charucoParams = new CharucoParameters();
        // detectParams
        // refineParams
        charucoParams.set_tryRefineMarkers(true);

        CharucoDetector detector = new CharucoDetector(board, charucoParams, detectParams, refineParams);
        ///

        final int waitTime = 30;
        final Size imgSize = new Size(cameraW, cameraH);

        /// video image capture

        // Get the UsbCamera from CameraServer
        final UsbCamera camera = CameraServer.startAutomaticCapture(camId);
        camera.setResolution(cameraW, cameraH);
        camera.setFPS(cameraFPS); // 30 max for lifecam
        camera.setExposureAuto();
           
        // Get a CvSink. This will capture Mats from the camera
        JavaCvSink cvSink = new JavaCvSink("sink1"); //CameraServer.getVideo();
        cvSink.setSource(camera);
        // Mats are very memory expensive. Lets reuse these.
        final Mat image = new Mat();
        final Mat grayMat = new Mat();
        ///
    
        List<Mat> allCharucoCorners = new ArrayList<>();
        List<Mat> allCharucoIds = new ArrayList<>();
        List<Mat> allImagePoints = new ArrayList<>();
        List<Mat> allObjectPoints = new ArrayList<>();
        // Detect charuco board from several viewpoints and fill
        // allCharucoCorners, allCharucoIds, allImagePoints and allObjectPoints
        while (!Thread.interrupted()) {
          if (cvSink.grabFrame(image) == 0) {
            System.out.println("grabFrame error " + cvSink.getError());
            continue; // skip the rest of the current iteration
          }

          final Mat imageCopy = new Mat();
          image.copyTo(imageCopy);
          
          // boolean monochrome = false; // commented out since it doesn't seem to add much to the program
          // if(monochrome)
          //   Core.extractChannel(image, grayMat, 0); // monochrome camera on 3 channels to 1 gray channel
          // else
            Imgproc.cvtColor(image, grayMat, Imgproc.COLOR_RGB2GRAY); // 3 channel color to 1 gray channel
    
          // System.out.println("image\n" + image);
    
          final Mat currentCharucoCorners = new Mat();
          final Mat currentCharucoIds = new Mat();
          final Mat currentObjectPoints = new Mat();
          final Mat currentImagePoints = new Mat();
          detector.detectBoard( image, currentCharucoCorners, currentCharucoIds );
    
          // if(printCount++ == 10)
          // {
            // printCount = 0;
            // System.out.format("%s %s%n rows %d, cols %d, channels %d, depth %d, dims %d, height %d, width %d, total %d%n",
            //                 "currentCharucoCorners", currentCharucoCorners,
            //                 currentCharucoCorners.rows(), currentCharucoCorners.cols(), currentCharucoCorners.channels(),
            //                 currentCharucoCorners.depth(), currentCharucoCorners.dims(), currentCharucoCorners.height(),
            //                 currentCharucoCorners.width(), currentCharucoCorners.total() );
            // System.out.format("%s %s%n rows %d, cols %d, channels %d, depth %d, dims %d, height %d, width %d, total %d%n",
            //                 "currentCharucoIds", currentCharucoIds,
            //                 currentCharucoIds.rows(), currentCharucoIds.cols(), currentCharucoIds.channels(),
            //                 currentCharucoIds.depth(), currentCharucoIds.dims(), currentCharucoIds.height(),
            //                 currentCharucoIds.width(), currentCharucoIds.total() );
            //currentCharucoCorners Mat [ 52*1*CV_32FC2, isCont=true, isSubmat=false, nativeObj=0x2dcf9573a80, dataAddr=0x2dcf95db4c0 ]
            // rows 52, cols 1, channels 2, depth 5, dims 2, height 52, width 1, total 52
            //currentCharucoIds Mat [ 52*1*CV_32SC1, isCont=true, isSubmat=false, nativeObj=0x2dcf9573460, dataAddr=0x2dcfe563c40 ]
            // rows 52, cols 1, channels 1, depth 4, dims 2, height 52, width 1, total 52
          // }
       
          Objdetect.drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
    
          Imgproc.putText(imageCopy, "Press 'c' to add; 'Esc' to finish",
          new Point(2, 13), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 0), 3);
          Imgproc.putText(imageCopy, "Press 'c' to add; 'Esc' to finish",
          new Point(2, 13), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
    
          HighGui.imshow("camera view", imageCopy);
          int key = HighGui.waitKey(waitTime); // wait so not beating on computer
    
          if (key == 27) break; // 'Esc' key pressed to stop capture and do the calibration
    
          // 'c' key pressed to capture view
          if(key == 67 && currentCharucoIds.total() >= 6) { // minimum number of points in view to be useful almost all the time
                                                            // opencv checks for 4 or 6 object points in different places
            System.out.println("\nCapture attempt with " + currentCharucoCorners.total() + " corners");
    
            final List<Mat> listCurrentCharucoCorners = new ArrayList<>();
            for(int i = 0; i < currentCharucoCorners.total(); i++) {
              listCurrentCharucoCorners.add(currentCharucoCorners.row(i)); // reformat the Mat for matchImagePoints
            }
    
            board.matchImagePoints(listCurrentCharucoCorners, currentCharucoIds,
                                       currentObjectPoints, currentImagePoints);
    
            // System.out.format("%s %s%n rows %d, cols %d, channels %d, depth %d, dims %d, height %d, width %d, total %d%n",
            // "currentObjectPoints", currentObjectPoints,
            // currentObjectPoints.rows(), currentObjectPoints.cols(), currentObjectPoints.channels(),
            // currentObjectPoints.depth(), currentObjectPoints.dims(), currentObjectPoints.height(),
            // currentObjectPoints.width(), currentObjectPoints.total() );
            // System.out.format("%s %s%n rows %d, cols %d, channels %d, depth %d, dims %d, height %d, width %d, total %d%n",
            // "currentImagePoints", currentImagePoints,
            // currentImagePoints.rows(), currentImagePoints.cols(), currentImagePoints.channels(),
            // currentImagePoints.depth(), currentImagePoints.dims(), currentImagePoints.height(),
            // currentImagePoints.width(), currentImagePoints.total() );
    
            if(currentObjectPoints.empty() || currentImagePoints.empty()) {
                System.out.println("Point matching failed; skipping this frame.");
                continue;
            }
            // allCharucoCorners.add(currentCharucoCorners);
            // allCharucoIds.add(currentCharucoIds);
            allObjectPoints.add(currentObjectPoints);
            allImagePoints.add(currentImagePoints);
            // allImages.add(image);
            // imageSize = image.size();
    
            System.out.println(allObjectPoints.size() + " frames captured");
    
          } // end this image capture
        } // end video captures
          
        /// HAVE THE DATA SO CALIBRATE CAMERA
        System.out.println("(slowly) CALIBRATING CAMERA");
        System.out.println(camera.getDescription());
        System.out.println(image.cols() + "x" + image.rows());
        System.out.flush(); // assure message appears that calibration has begun
        final Mat cameraMatrix;
        final Mat distCoeffs = new Mat();
        final List<Mat> rvecs = new ArrayList<>();
        final List<Mat> tvecs = new ArrayList<>();

        // if(calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
          // cameraMatrix.at<double>(0, 0) = aspectRatio;
        // }

        try
        {
          final double repError = Calib3d.calibrateCamera(allObjectPoints, allImagePoints, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs );
          System.out.println("camera matrix " + cameraMatrix + "\n" + cameraMatrix.dump());
          System.out.println("distortion coefficients " + distCoeffs + "\n" + distCoeffs.dump());
          System.out.println("repError " + repError);      
        }
        catch(CvException error)
        {
          System.out.println(error);
          
          if(error.toString().contains("DBL_EPSILON"))
          {
            System.out.println("\nPossibly had too many frames. Try < 190 frames.");
          }

          if(error.toString().contains("nimages > 0"))
          {
            System.out.println("\nNo frames. Try again and press 'c' to add frames.");
          }
        }

        System.exit(0);
    }
}
// C:\Users\RKT\frc\FRC2023\Code\Java> .\gradlew shadowJar

// BUILD SUCCESSFUL in 6s
// 8 actionable tasks: 3 executed, 5 up-to-date

// C:\Users\RKT\frc\FRC2023\Code\Java> java -jar build\libs\TestApplication-winx64.jar

// CS: USB Camera 1: Connecting to USB camera on \\?\ usb#vid_0c45&pid_6366&mi_00#8&2355927a&2&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global
// CS: USB Camera 1: Disconnected from \\?\ usb#vid_0c45&pid_6366&mi_00#8&2355927a&2&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global
// CS: USB Camera 1: Connecting to USB camera on \\?\ usb#vid_0c45&pid_6366&mi_00#8&2355927a&2&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global
// CS: USB Camera 1: Disconnected from \\?\ usb#vid_0c45&pid_6366&mi_00#8&2355927a&2&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global
// CS: USB Camera 1: Connecting to USB camera on \\?\ usb#vid_0c45&pid_6366&mi_00#8&2355927a&2&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global
// grabFrame error timed out getting frame

// Capture attempt with 17 corners
// 1 frames captured
// .
// .
// .
// 114 frames captured
// (slowly) CALIBRATING CAMERA
// Arducam OV9281 USB Camera (1)
// 320x240
// camera matrix Mat [ 3*3*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x1a66b813900, dataAddr=0x1a66e5c53c0 ]
// [229.6626080788598, 0, 164.8030509174101;
//  0, 228.18269360966, 114.3647984489618;
//  0, 0, 1]
// distortion coefficients Mat [ 1*5*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x1a66b813ba0, dataAddr=0x1a66c32fb40 ]
// [-0.09066001489984649, 0.203254195450917, -0.00932087456854446, 0.007449679350668339, -0.2048258601718673]
// repError 3.288901566607536

/*
System.out.println(dictionary.get_bytesList().total());
System.out.println(dictionary.get_bytesList() + "\n" + dictionary.get_bytesList().dump());
System.out.println(Dictionary.getBitsFromByteList(dictionary.get_bytesList().row(0), 5).dump());
*/

/*
// from the OpenCV program for debugging display all the points found
void drawDetectedCornersCharuco(InputOutputArray _image, InputArray _charucoCorners,
                                InputArray _charucoIds, Scalar cornerColor) {
    CV_Assert(!_image.getMat().empty() &&
              (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
    CV_Assert((_charucoCorners.getMat().total() == _charucoIds.getMat().total()) ||
              _charucoIds.getMat().total() == 0);

    size_t nCorners = _charucoCorners.getMat().total();
    for(size_t i = 0; i < nCorners; i++) {
        Point2f corner = _charucoCorners.getMat().at<Point2f>((int)i);
        // draw first corner mark
        rectangle(_image, corner - Point2f(3, 3), corner + Point2f(3, 3), cornerColor, 1, LINE_AA);
        // draw ID
        if(!_charucoIds.empty()) {
            int id = _charucoIds.getMat().at<int>((int)i);
            stringstream s;
            s << "id=" << id;
            putText(_image, s.str(), corner + Point2f(5, -5), FONT_HERSHEY_SIMPLEX, 0.5,
                    cornerColor, 2);
        }
    }
}
 */

// using opencv image input instead of WPILib camera server
//     System.loadLibrary("opencv_videoio_ffmpeg480_64"); required maybe. dll must be in PATH
//     VideoCapture cap = new VideoCapture(); required
    // video parameters
    // MatOfInt videoParams = new MatOfInt( // pair-wise; param1, value1, ...
    //   Videoio.CAP_PROP_AUTO_EXPOSURE, 1,
    //       Videoio.CAP_PROP_FPS, 100,
    //       Videoio.CAP_PROP_FRAME_HEIGHT, 240,
    //       Videoio.CAP_PROP_FRAME_WIDTH, 320 );
    // cap.open(camId/*, Videoio.CAP_ANY, videoParams*/);
    // cap.set(Videoio.CAP_PROP_FORMAT, CvType.CV_8UC(3));
//    cap.open("http://127.0.0.1:1181/stream.mjpg"); required
	  //    cap.open("http://127.0.0.1:1181/stream.mjpg", Videoio.CAP_FFMPEG);
    // System.out.println(cap.set(Videoio.CAP_PROP_FOURCC, VideoWriter.fourcc('M','J','P','G')));
//    if (cap.isOpened()) cap.read(image); required
