// Based on https://github.com/wpilibsuite/StandaloneAppSamples/tree/main/Java
// and JavaCvSink.java from WPILib ShuffleBoard
// and OpenCV

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

/**
 * Program
 */
public class Program {
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

        // var inst = NetworkTableInstance.getDefault();

        // int printCount = 0;

        /// Charco Board
        // print the board then measure it carefully and put those measurements below
        Dictionary dictionary = Objdetect.getPredefinedDictionary(Objdetect.DICT_5X5_1000);
        int squaresX = 11;
        int squaresY = 8;
        float squareLength = /*0.015f;*/ .018536f;
        float markerLength = /*0.012f;*/ .01486f;
        CharucoBoard board = new CharucoBoard(new Size(squaresX, squaresY), squareLength, markerLength, dictionary);
        Mat boardImage = new Mat();
        Size boardImageSize = new Size(1280, 720);
        board.generateImage(boardImageSize, boardImage);

        boolean writeBoard = false;
        if(writeBoard)
        {
          // write ChArUco Board to print and use for calibration
          MatOfInt writeBoardParams = new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, 100); // pair-wise; param1, value1, ...
          String file = "ChArUcoBoard.jpg";
          Imgcodecs.imwrite(
            file,
            boardImage,
            writeBoardParams);
          System.exit(0);
        }
        ///

        /// detector parameters
        DetectorParameters detectParams = new DetectorParameters();
        RefineParameters refineParams = new RefineParameters();
        CharucoParameters charucoParams = new CharucoParameters();
        // detectParams
        // refineParams
        charucoParams.set_tryRefineMarkers(true);

        CharucoDetector detector = new CharucoDetector(board, charucoParams, detectParams, refineParams);
        ///

        /// video image capture
        int camId = 1; // Checks for the specified camera and uses it if present. 0 internal, 1 external if there is a 0 internal (sometimes)
        int waitTime = 30;

        int cameraW = 320;
        int cameraH = 240;
        Size
          // imgSize = new Size(1280, 720);
          // imgSize = new Size(640, 480);
          imgSize = new Size(cameraW, cameraH);


        // Get the UsbCamera from CameraServer
        UsbCamera camera = CameraServer.startAutomaticCapture(camId);
        // USB\VID_04F2&PID_B230&MI_00\7&369E123A&8&0000 laptop internal path
        // USB\VID_0C45&PID_6366&MI_00\8&2355927A&2&0000 arducam
        // Set the resolution and frames per second
        camera.setResolution(cameraW, cameraH);
        camera.setFPS(121); // 30 max for lifecam
        camera.setExposureAuto();

        // CvSource outputStream = CameraServer.putVideo("Detected", cameraW, cameraH);
        
        // Get a CvSink. This will capture Mats from the camera
        JavaCvSink cvSink = new JavaCvSink("sink1"); //CameraServer.getVideo();
        cvSink.setSource(camera);
        // Mats are very memory expensive. Lets reuse these.
        Mat image = new Mat();
        var grayMat = new Mat();
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

          Mat imageCopy = new Mat();
          image.copyTo(imageCopy);
          
          boolean monochrome = false;
    
          if(monochrome)
            Core.extractChannel(image, grayMat, 0); // monochrome camera on 3 channels to 1 gray channel
          else
            Imgproc.cvtColor(image, grayMat, Imgproc.COLOR_RGB2GRAY); // color camera to  1 gray channel
    
          // System.out.println("image\n" + image);
    
          Mat currentCharucoCorners = new Mat();
          Mat currentCharucoIds = new Mat();
          Mat currentObjectPoints = new Mat();
          Mat currentImagePoints = new Mat();
          detector.detectBoard( image, currentCharucoCorners, currentCharucoIds );
    
          // if(printCount++ == 10)
          // {
            // printCount = 0;
            // System.out.println("."); // for the riolog to print a "blank" line
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
          new Point(10, 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 0), 2);
          Imgproc.putText(imageCopy, "Press 'c' to add; 'Esc' to finish",
          new Point(10, 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 255), 1);
    
          // HighGui.imshow("ChArUcoBoard", boardImage);
          HighGui.imshow("out", imageCopy);
          int key = HighGui.waitKey(waitTime); // wait so not beating on computer
    
          if (key == 27) break; // stop capture and do the calibration
    
          // 'c' key pressed to capture view
          if(key == 67 && currentCharucoIds.total() >= 4) { // minimum number of points in view to be useful
    
            System.out.println("\nCapture attempt with " + currentCharucoCorners.total() + " corners");
    
            List<Mat> listCurrentCharucoCorners = new ArrayList<>();
            for(int i = 0; i < currentCharucoCorners.total(); i++) {
              listCurrentCharucoCorners.add(currentCharucoCorners.row(i));
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
    
            if(currentImagePoints.empty() || currentObjectPoints.empty()) {
                System.out.println("Point matching failed; skipping this frame.");
                continue;
            }
            // allCharucoCorners.add(currentCharucoCorners);
            // allCharucoIds.add(currentCharucoIds);
            allImagePoints.add(currentImagePoints);
            allObjectPoints.add(currentObjectPoints);
            // allImages.add(image);
            // imageSize = image.size();
    
            System.out.println(allImagePoints.size() + " frames captured");
    
          } // end this image capture
        } // end video captures
          
        // HAVE THE DATA SO CALIBRATE CAMERA
        if(allObjectPoints.size() < 6 || allImagePoints.size() < 6) // need at least 6 views for calibrateCamera DLT algorithm
        {
          System.out.println("Need at least 6 views; try again from the beginning.");
          System.exit(1);
        }

        System.out.println("(slowly) CALIBRATING CAMERA");
        System.out.println(image.cols() + "x" + image.rows());
        System.out.flush();
        Mat cameraMatrix;
        Mat distCoeffs = new Mat();
        List<Mat> rvecs = new ArrayList<>();
        List<Mat> tvecs = new ArrayList<>();

        // if(calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
          // cameraMatrix.at<double>(0, 0) = aspectRatio;
        // }
        double repError = Calib3d.calibrateCamera(allObjectPoints, allImagePoints, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs );

        System.out.println("camera matrix " + cameraMatrix + "\n" + cameraMatrix.dump());
        System.out.println("distortion coefficients " + distCoeffs + "\n" + distCoeffs.dump());

        System.out.println("repError " + repError);

        System.exit(0);
    }
}

// PS C:\Users\RKT\frc\FRC2023\Code\Java> .\gradlew shadowJar

// BUILD SUCCESSFUL in 6s
// 8 actionable tasks: 3 executed, 5 up-to-date
// PS C:\Users\RKT\frc\FRC2023\Code\Java> java -jar build\libs\TestApplication-winx64.jar
// CS: USB Camera 1: Connecting to USB camera on \\?\ usb#vid_0c45&pid_6366&mi_00#8&2355927a&2&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global
// CS: USB Camera 1: Disconnected from \\?\ usb#vid_0c45&pid_6366&mi_00#8&2355927a&2&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global
// CS: USB Camera 1: Connecting to USB camera on \\?\ usb#vid_0c45&pid_6366&mi_00#8&2355927a&2&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global
// CS: USB Camera 1: Disconnected from \\?\ usb#vid_0c45&pid_6366&mi_00#8&2355927a&2&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global
// CS: USB Camera 1: Connecting to USB camera on \\?\ usb#vid_0c45&pid_6366&mi_00#8&2355927a&2&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global
// grabFrame error timed out getting frame

// Capture attempt with 17 corners
// 1 frames captured

// Capture attempt with 12 corners
// 2 frames captured

// Capture attempt with 7 corners
// 3 frames captured

// Capture attempt with 11 corners
// 4 frames captured

// Capture attempt with 7 corners
// 5 frames captured

// Capture attempt with 5 corners
// 6 frames captured

// Capture attempt with 4 corners
// 7 frames captured

// Capture attempt with 5 corners
// 8 frames captured

// Capture attempt with 36 corners
// 9 frames captured

// Capture attempt with 16 corners
// 10 frames captured

// Capture attempt with 13 corners
// 11 frames captured
// (slowly) CALIBRATING CAMERA
// 320x240
// camera matrix Mat [ 3*3*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x2642a7eb6f0, dataAddr=0x2642a7f0dc0 ]
// [277.8609536766253, 0, 152.9657491150284;
//  0, 277.2565407471934, 117.7440061323202;
//  0, 0, 1]
// distortion coefficients Mat [ 1*5*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x2642a7eb5a0, dataAddr=0x2642a623200 ]
// [0.5171960321534104, -5.745370954625806, -0.003966015723441144, 0.003863082859973825, 20.16955092184625]
// repError 0.3952622420671669

/*
System.out.println(dictionary.get_bytesList().total());
System.out.println(dictionary.get_bytesList() + "\n" + dictionary.get_bytesList().dump());
System.out.println(Dictionary.getBitsFromByteList(dictionary.get_bytesList().row(0), 5).dump());
*/
/*
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