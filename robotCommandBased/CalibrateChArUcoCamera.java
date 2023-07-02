// https://docs.opencv.org/4.7.0/da/d13/tutorial_aruco_calibration.html
// C:\Users\RKT\frc\FRC2023\opencv_contrib-4.x\modules\aruco\samples\calibrate_camera_charuco.cpp
// or calibrate_camera.cpp

import java.util.ArrayList;
import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CharucoBoard;
import org.opencv.objdetect.CharucoDetector;
import org.opencv.objdetect.CharucoParameters;
import org.opencv.objdetect.DetectorParameters;
import org.opencv.objdetect.Dictionary;
import org.opencv.objdetect.Objdetect;
import org.opencv.objdetect.RefineParameters;
import org.opencv.videoio.VideoCapture;

public class App {
  static {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME); // Load the native OpenCV library
    // System.out.println("OpenCV version " + Core.getVersionString() + "\n" + Core.getBuildInformation());
  }

  /**
   * @param args
   * @throws Exception
   */
  public static void main(String[] args) throws Exception {
    int printCount = 0;
    /// Charco Board
    Dictionary dictionary = Objdetect.getPredefinedDictionary(Objdetect.DICT_5X5_1000);
    int squaresX = 11;
    int squaresY = 8;
    float squareLength = 0.015f;
    float markerLength = 0.012f;
    CharucoBoard board = new CharucoBoard(new Size(squaresX, squaresY), squareLength, markerLength, dictionary);
    Mat boardImage = new Mat();
    Size boardImageSize = new Size(1280, 720);
    board.generateImage(boardImageSize, boardImage);
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
    Mat image = new Mat();
    int camId = 1; // Checks for the specified camera and uses it if present. 0 internal 1 external if there is a 0 internal (sometimes)
    VideoCapture cap;
    int waitTime = 200;
    cap = new VideoCapture();
    Size
      imgSize = new Size(1280, 720);
      // imgSize = new Size(640, 480);
    cap.open(camId);
    if( cap.isOpened() ) {
        cap.read(image);
    }
    else {
        System.out.println("No camera");
    }
    var grayMat = new Mat();
    ///

    List<Mat> allCharucoCorners = new ArrayList<>();
    List<Mat> allCharucoIds = new ArrayList<>();
    List<Mat> allImagePoints = new ArrayList<>();
    List<Mat> allObjectPoints = new ArrayList<>();
    // Detect charuco board from several viewpoints and fill
    // allCharucoCorners, allCharucoIds, allImagePoints and allObjectPoints
    while(!Thread.interrupted()) {
      if (cap.isOpened()) cap.read(image);

      System.out.println(image);

      Mat imageCopy = new Mat();
      image.copyTo(imageCopy);
      
      boolean monchrome = false;

      if(monchrome)
        Core.extractChannel(image, grayMat, 0); // monochrome camera on 3 channels to 1 gray channel
      else
        Imgproc.cvtColor(image, grayMat, Imgproc.COLOR_RGB2GRAY); // color camera to  1 gray channel

      Mat currentCharucoCorners = new Mat();
      Mat currentCharucoIds = new Mat();
      Mat currentObjectPoints = new Mat();
      Mat currentImagePoints = new Mat();
      detector.detectBoard( image, currentCharucoCorners, currentCharucoIds );
      
      if(currentCharucoIds.total() == 0)
      {
      System.out.println("image\n" + image);
      System.out.println(".\ncurrentCharucoCorners\n" + currentCharucoCorners + "\ncurrentCharucoIds\n" + currentCharucoIds);
      System.out.println(currentCharucoIds.dump());
      System.out.println(currentCharucoCorners.dump());
        continue;
      }
        //   Core.extractChannel(mat, grayMat, 0); // monochrome camera on 3 channels to 1 gray channel
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();
        List<Mat> rejectedCorners = new ArrayList<>();

        // detector.detectBoard(mat, currentCharucoCorners, currentCharucoIds, corners, ids);
        // detector.detectMarkers(grayMat, corners, ids);

      // board.matchImagePoints(allCharucoCorners, currentCharucoIds, currentObjectPoints, currentImagePoints );

      //   detector.detectMarkers(grayMat, corners, ids);

      if(printCount++ == 1)
      {
        printCount = 0;
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
      }
      Objdetect.drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);

      Imgproc.putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
      new Point(10, 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 255), 1);

      // HighGui.imshow("ChArUcoBoard", boardImage);
      HighGui.imshow("out", imageCopy);
      int key = HighGui.waitKey(waitTime); // wait so not beating on computer

      if (key == 27) break; // end program if key is pressed in an OpenCV window, need at least 6 views for calibrateCamera DLT algorithm

      // 'c' key pressed to capture view
      if(key == 67 && currentCharucoIds.total() >= 4) { // minimum number of points in view to be useful

        System.out.println("\n\ncapture attempt with # corners " + currentCharucoCorners.total());

        List<Mat> listCurrentCharucoCorners = new ArrayList<>();
        for(int i = 0; i < currentCharucoCorners.total(); i++) {
          listCurrentCharucoCorners.add(currentCharucoCorners.row(i));
        }

        board.matchImagePoints(listCurrentCharucoCorners, currentCharucoIds,
                                   currentObjectPoints, currentImagePoints);

        System.out.format("%s %s%n rows %d, cols %d, channels %d, depth %d, dims %d, height %d, width %d, total %d%n",
        "currentObjectPoints", currentObjectPoints,
        currentObjectPoints.rows(), currentObjectPoints.cols(), currentObjectPoints.channels(),
        currentObjectPoints.depth(), currentObjectPoints.dims(), currentObjectPoints.height(),
        currentObjectPoints.width(), currentObjectPoints.total() );
        System.out.format("%s %s%n rows %d, cols %d, channels %d, depth %d, dims %d, height %d, width %d, total %d%n",
        "currentImagePoints", currentImagePoints,
        currentImagePoints.rows(), currentImagePoints.cols(), currentImagePoints.channels(),
        currentImagePoints.depth(), currentImagePoints.dims(), currentImagePoints.height(),
        currentImagePoints.width(), currentImagePoints.total() );

        if(currentImagePoints.empty() || currentObjectPoints.empty()) {
            System.out.println("Point matching failed, try again.");
            continue;
        }

        System.out.println("Frame captured");

        // allCharucoCorners.add(currentCharucoCorners);
        // allCharucoIds.add(currentCharucoIds);
        allImagePoints.add(currentImagePoints);
        allObjectPoints.add(currentObjectPoints);
        // allImages.add(image);
        // imageSize = image.size();
      } // end this image capture
      
    } // end video captures
// double calibrateCamera(List<Mat> objectPoints, List<Mat> imagePoints, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs)
      System.out.println("CALIBRATING CAMERA");
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

      System.out.println("repError " + repError);
  } // main
} // app class

/*
capture attempt with # corners 33
currentObjectPoints Mat [ 33*1*CV_32FC3, isCont=true, isSubmat=false, nativeObj=0x1acfe38ede0, dataAddr=0x1acfa680e00 ]
 rows 33, cols 1, channels 3, depth 5, dims 2, height 33, width 1, total 33
currentImagePoints Mat [ 33*1*CV_32FC2, isCont=true, isSubmat=false, nativeObj=0x1acfe390430, dataAddr=0x1acfa7e9540 ]
 rows 33, cols 1, channels 2, depth 5, dims 2, height 33, width 1, total 33
Frame captured
CALIBRATING CAMERA
camera matrix Mat [ 3*3*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x1acfe3a2670, dataAddr=0x1acfe4a2f40 ]
[1880.130015393942, 0, 748.6946814242159;
 0, 1685.763656857004, 416.1560853827729;
 0, 0, 1]
repError 35.206154068700435


Mat [ 720*1280*CV_8UC3, isCont=true, isSubmat=false, nativeObj=0x1de60c9c4f0, dataAddr=0x1de63c40080 ]
CALIBRATING CAMERA
float[] cameraParm = {(float)cameraFx,   0.f,             (float)cameraCx,
                                0.f,             (float)cameraFy, (float)cameraCy,
                                0.f,             0.f,             1.f};
camera matrix Mat [ 3*3*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x1de60f02860, dataAddr=0x1de6482ebc0 ]
[129.238350926411, 0, 545.8079157023064;
 0, 485.867442369393, 306.1309597922968;
 0, 0, 1]
repError 34.945444340610635
*/

// Set calibration flags (same than in calibrateCamera() function)
// int calibrationFlags = ...;

// double repError = calibrateCamera(
//  allObjectPoints, allImagePoints, imgSize,
//  cameraMatrix, distCoeffs, rvecs, tvecs, noArray(),
//  noArray(), noArray(), calibrationFlags=0
// );

// Mat boardImage;
// board.draw(new Size(600, 500), boardImage, 10, 1);
// imwrite("BoardImage.jpg", boardImage);

// camera.setResolution(cameraW, cameraH);
// camera.setFPS(30); // 30 for lifecam
// camera.setExposureAuto();

//     // System.out.println("ids\n" + ids.dump());
//     detector.refineDetectedMarkers(grayMat, board, corners, ids, rejectedCorners);
// // corners [Mat [ 1*4*CV_32FC2
//     Mat Corners = Converters.vector_Mat_to_Mat(corners); // for draw

/*
   // write a test file to imread
    MatOfInt parameters = new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, 100); // pair-wise; param1, value1, ...
    String file = "DiagonalLine.jpg";
    Imgcodecs.imwrite(
      file,
      Line(),
      parameters);

System.out.println(dictionary.get_bytesList().total());
System.out.println(dictionary.get_bytesList() + "\n" + dictionary.get_bytesList().dump());
System.out.println(Dictionary.getBitsFromByteList(dictionary.get_bytesList().row(0), 5).dump());
*/