package frc.robot.Sensors;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Spliterator;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class ApriltagVisionThreadProc implements Runnable {
    
public void run() {
    System.out.println("ApriltagVisionThreadProc");

    var detector = new AprilTagDetector();
    // look for tag16h5, don't correct any error bits
    detector.addFamily("tag16h5", 0);

  // Cam position
  Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));

  // Tag positions
  AprilTagFieldLayout aprilTagFieldLayout = null;

  try {
    aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
  } catch (IOException e) {
    e.printStackTrace();
  }

  /*
   *  [AprilTag(ID: 1, pose: Pose3d(Translation3d(X: 15.51, Y: 1.07, Z: 0.46), Rotation3d(Quaternion(0.0, 0.0, 0.0, 1.0)))), AprilTag(ID: 2, pose: Pose3d(Translation3d(X: 15.51, Y: 2.75, Z: 0.46), Rotation3d(Quaternion(0.0, 0.0, 0.0, 1.0)))), AprilTag(ID: 3, pose: Pose3d(Translation3d(X: 15.51, Y: 4.42, Z: 0.46), Rotation3d(Quaternion(0.0, 0.0, 0.0, 1.0)))), AprilTag(ID: 4, pose: Pose3d(Translation3d(X: 16.18, Y: 6.75, Z: 0.70), Rotation3d(Quaternion(0.0, 0.0, 0.0, 1.0)))), AprilTag(ID: 5, pose: Pose3d(Translation3d(X: 0.36, Y: 6.75, Z: 0.70), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0)))), AprilTag(ID: 6, pose: Pose3d(Translation3d(X: 1.03, Y: 4.42, Z: 0.46), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0)))), AprilTag(ID: 7, pose: Pose3d(Translation3d(X: 1.03, Y: 2.75, Z: 0.46), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0)))), AprilTag(ID: 8, pose: Pose3d(Translation3d(X: 1.03, Y: 1.07, Z: 0.46), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))]
   */

  System.out.println("Tags on file");    // System.out.println(aprilTagFieldLayout.getTags());
	Spliterator<AprilTag> namesSpliterator = aprilTagFieldLayout.getTags().spliterator();  // Getting Spliterator
	namesSpliterator.forEachRemaining(System.out::println);		// Traversing and printing elements
  
    // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
    // fx camera horizontal focal length, in pixels
    // fy camera vertical focal length, in pixels
    // cx camera horizontal focal center, in pixels
    // cy camera vertical focal center, in pixels
    double[] cameraConfig = {699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522};
    // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
    var poseEstConfig =
        new AprilTagPoseEstimator.Config( // tag and camera configuration
            0.1524, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
    var estimator = new AprilTagPoseEstimator(poseEstConfig);

    // Get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture(); // http://10.42.37.2:1181/   http://roborio-4237-frc.local:1181/?action=stream
    // Set the resolution
    camera.setResolution(640, 480);

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("Detected", 640, 480); // http://10.42.37.2:1182/  http://roborio-4237-frc.local:1182/?action=stream

    // Mats are very memory expensive. Lets reuse these.
    var mat = new Mat();
    var grayMat = new Mat();

    // Instantiate once
    ArrayList<Long> tags = new ArrayList<>();
    var outlineColor = new Scalar(0, 255, 0);
    var crossColor = new Scalar(0, 0, 255);

    var pillarColor = new Scalar(0, 255, 255);

    // We'll output to NT
    NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
    IntegerArrayPublisher pubTags = tagsTable.getIntegerArrayTopic("tags").publish();

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

      Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

      AprilTagDetection[] detections = detector.detect(grayMat);

      // have not seen any tags yet
      tags.clear();

      for (AprilTagDetection detection : detections) {
        
        if(detection.getId() != 1) // margin < 20 seems bad  > 140 are good maybe > 50?
        {
           System.out.println("bad id " + detection.getId() + " " + detection.getDecisionMargin() + " " + detection.getHamming());
           break;
        }

        System.out.println("good id " + detection.getId() + " " + detection.getDecisionMargin() + " " + detection.getHamming());


        // remember we saw this tag
        tags.add((long) detection.getId());

        { // draw lines around the tag
        for (var i = 0; i <= 3; i++) {
          var j = (i + 1) % 4;
          var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
          var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
          Imgproc.line(mat, pt1, pt2, outlineColor, 2);
        }

        // mark the center of the tag
        var cx = detection.getCenterX();
        var cy = detection.getCenterY();
        var ll = 10;
        Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), crossColor, 2);
        Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), crossColor, 2);

        // identify the tag
        Imgproc.putText(
            mat,
            Integer.toString(detection.getId()),
            new Point(cx + ll, cy),
            Imgproc.FONT_HERSHEY_SIMPLEX,
            1,
            crossColor,
            3);
      } // end draw lines around the tag

        // determine pose
        Transform3d pose = estimator.estimate(detection);

/* attempting to draw a 3-D box in front of the AprilTag */

        Mat H = new Mat(3, 3, CvType.CV_32F);
        H.put(0, 0, detection.getHomography());
        // System.out.println("H from AprilTag detector\n" + H.dump());

        // original, before distortion corner locations
        MatOfPoint2f obj = new MatOfPoint2f(new Point(-1.,1.), new Point(1., 1.), new Point(1., -1.), new Point(-1., -1.));

    // distorted by perspective corner locations found by AprilTag detector
    // reformating to use in OpenCV solvePnP to get the homography that works
    MatOfPoint2f scene = new MatOfPoint2f(
      new Point(detection.getCornerX(0), detection.getCornerY(0)),
      new Point(detection.getCornerX(1), detection.getCornerY(1)),
      new Point(detection.getCornerX(2), detection.getCornerY(2)),
      new Point(detection.getCornerX(3), detection.getCornerY(3))
    );

    // Mat hg = Calib3d.findHomography(obj, scene); // verified the same as WPILib calc
    // System.out.println("H from corner points (found by AprilTag detector)\n" + hg.dump());
    /*
    H from tag
    [144.80396, 12.976685, 350.0528;
    41.401249, 145.22011, 297.9584;
    0.16224369, 0.012986111, 1]
    H from points
    [144.8039564583409, 12.97668437929864, 350.0528031907195;
    41.40125080826242, 145.2201143644582, 297.9583956188442;
    0.1622436868404965, 0.01298611046976967, 1]
    */

        // camera same as above but different format for OpenCV
        float[] cameraParm = {(float)cameraConfig[0],   0.f,              (float)cameraConfig[2],
                               0.f,               (float)cameraConfig[1], (float)cameraConfig[3],
                               0.f,                      0.f,                1.f};   
        Mat K = new Mat(3, 3, CvType.CV_32F); // camera matrix
        K.put(0, 0, cameraParm);
        // System.out.println("K\n" + K.dump());

        MatOfDouble distCoeffs = new MatOfDouble(); // not using any camera distortions so it's empty

        // 3D points of ideal, original corners
        MatOfPoint3f bottom = new MatOfPoint3f(
          new Point3(-1.,1., 0.),
              new Point3(1., 1., 0.),
              new Point3(1., -1., 0.),
              new Point3(-1., -1., 0.));
        // System.out.println("bottom\n" + bottom.dump());
        
        MatOfPoint3f top = new MatOfPoint3f(
          new Point3(-1.,1., -0.621),
              new Point3(1., 1., -0.621),
              new Point3(1., -1., -0.621),
              new Point3(-1., -1., -0.621));
            // System.out.println("top\n" + top.dump());

        Mat R = new Mat();
        Mat T = new Mat();
        Calib3d.solvePnP(bottom, scene, K, distCoeffs, R, T);

        // System.out.println("R\n" + R.dump());
        // System.out.println("T\n" + T.dump());

        MatOfPoint2f imagePointsBottom = new MatOfPoint2f();
        Calib3d.projectPoints(bottom, R, T, K, distCoeffs, imagePointsBottom);
        // System.out.println("bottom projected\n" + imagePointsBottom.dump());

        MatOfPoint2f imagePointsTop = new MatOfPoint2f();
        //{
        // decomposing the WPILib homography for rotations and translations yields bad results. For some reason
        // the R and T aren't suitable as is for use to projectPoints. Some of the multiple solutions almost look
        // right but are too long or far from the object. Lacking understanding of this problem, it easier to
        // redo the homography and get the R and T again from SolvePnP instead of from the WPILib calc.
        //  // get the rotations and translations from homography and camera because not allOpenCV methods use H, K
        //  List<Mat> rotations = new ArrayList<Mat>();
        //  List<Mat> translations = new ArrayList<Mat>();
        //  List<Mat> normals = new ArrayList<Mat>();
        //  Calib3d.decomposeHomographyMat(H, K, rotations, translations, normals);
        //  // may be several poses in list from which to choose - showing getting "0"
        //  // System.out.println("rotations " + rotations.size() + " " + rotations.get(0).dump());
        //  // System.out.println("translations " + translations.size() + " " + translations.get(0).dump());
        //  // System.out.println("normals " + normals.size() + " " + normals.get(0).dump());
        // Calib3d.projectPoints(top,
        //         rotations.get(0), translations.get(0), K, distCoeffs, imagePointsTop);
        // // System.out.println("top projected version 1\n" + imagePoints.dump());
        //}

        Calib3d.projectPoints(top, R, T, K, distCoeffs, imagePointsTop); // good from solvePnP
        // System.out.println("top projected version 2\n" + imagePointsTop.dump());
        
        // gyrations to use polyline.
        // Its input is any number of paths each with any number of points for line segments
        // Here is plotting only one path with few points
        // make the list of Points
        // put that list of Points into a Mat for one Line
        // put that Mat of one Line into a list of Mats
        
        ArrayList<Point> topCornerPoints = new ArrayList<Point>();

        // draw from bottom points to top points - pillars
        for(int i = 0; i < 4; i++)
        {
          double x1, y1, x2, y2;
          x1 = imagePointsBottom.get(i, 0)[0];
          y1 = imagePointsBottom.get(i, 0)[1];
          x2 = imagePointsTop.get(i, 0)[0];
          y2 = imagePointsTop.get(i, 0)[1];

          topCornerPoints.add(new Point(x2, y2));

          // System.out.format("%6.0f%6.0f%6.0f%6.0f\n",x1, y1, x2,y2);
          Imgproc.line(mat,
            new Point(x1, y1),
            new Point(x2, y2),
            crossColor,
            2);
        }

        MatOfPoint topCornersTemp = new MatOfPoint();
        topCornersTemp.fromList(topCornerPoints);

        ArrayList<MatOfPoint> topCorners = new ArrayList<MatOfPoint>();
        topCorners.add(topCornersTemp);
       
        Imgproc.polylines(mat, topCorners, true, crossColor, 2);

/*
good id 1 168.02635
H from AprilTag detector
[152.66724, 22.536674, 444.27634;
28.07736, 153.66794, 263.69174;
0.14408411, 0.068441071, 1]
H from corner points (found by AprilTag detector)
[152.6672438246534, 22.53667496963242, 444.276335839681;
28.07736013104226, 153.6679378100173, 263.6917461028095;
0.1440841076637523, 0.06844107325490739, 1]
bottom
[-1, 1, 0;
1, 1, 0;
1, -1, 0;
-1, -1, 0]
top
[-1, 1, 0.20999999;
1, 1, 0.20999999;
1, -1, 0.20999999;
-1, -1, 0.20999999]
top projected version 1
[-155.45309, 983.91138;
1126.9266, 1053.5862;
627.26904, -79.857544;
-19.112583, 134.07855]
R
[0.06917061174969411;
-0.7564875824324934;
0.01280035479286561]
T
[0.6963923078108443;
0.3533217937187608;
4.940283351843335]
bottom projected
[334.6701, 421.54391;
516.26752, 366.60089;
529.00525, 127.08693;
346.54572, 105.21191]
top projected version 2
[312.49637, 412.08481;
494.57004, 360.79922;
506.52805, 127.54261;
323.36362, 106.57743]
 
good id 1 85.965904
K
[699.37781, 0, 345.60593;
0, 677.71613, 207.12741;
0, 0, 1]
rotations 4 [0.1288397008036668, -0.74578768111936, 0.6536062011540467;
-0.8426664804893775, 0.2651331565542652, 0.4686337716758675;
-0.5227939691272177, -0.6111506720811555, -0.5942906038781306]
translations 4 [1.232439733406374;
1.198923387300751;
-0.2597908138797531]
normals 4 [0.8703295269160756;
0.4845392100874308;
-0.08802424930602196]
X [-2445.2781, -2213.5486]
Y [-2177.8201, -2380.8533]
Z [-3728.1716, -3689.6724]
*/
        // H homography

        // K camera
        // fx fy cx c_y 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522
        // fx 0 cx
        // 0 fy cy
        // 0  0  1

        // put pose into dashbaord
        // pose.getX() same as pose.getTranslation().getX()
        // Rotation3d rot = pose.getRotation();

        // Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
        //   target.getBestCameraToTarget(), 
        //   aprilTagFieldLayout.getTagPose(target.getFiducialId()),
        //   robotToCam);  

        tagsTable
            .getEntry("pose_" + detection.getId())
            .setDoubleArray(
                new double[] {
                  pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getTranslation().getZ()
                  ,
                  pose.getRotation().getQuaternion().getX(), pose.getRotation().getQuaternion().getY(), pose.getRotation().getQuaternion().getZ(), pose.getRotation().getQuaternion().getW()
                });
        // tagsTable
        //     .getEntry("pose_" + detection.getId())
        //     .setDoubleArray(
        //         new double[] {
        //           pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getTranslation().getZ()
        //           ,
        //           pose.getRotation().getX(), pose.getRotation().getY(), pose.getRotation().getZ()
        //         });

        // System.out.format("%,6f %,6f %,6f %,6f %,6f %,6f\n",
        //             pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getTranslation().getZ()
        //             ,
        //             pose.getRotation().getX(), pose.getRotation().getY(), pose.getRotation().getZ()
        // );
        
        Imgproc.rectangle(mat, new Point(0., 0.), new Point(640., 50.), new Scalar(255., 255., 255.), -1);

        Imgproc.putText(mat,
         String.format("pose (x, y, z meters) %,6.2f %,6.2f %,6.2f",
          pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getTranslation().getZ()),
          new Point(0., 25.), Imgproc.FONT_HERSHEY_SIMPLEX,1., new Scalar(255., 0., 255.));

        Imgproc.putText(mat,
          String.format("angle (Tx, Ty, Tz rads) %,6.2f %,6.2f %,6.2f",
            pose.getRotation().getX(), pose.getRotation().getY(), pose.getRotation().getZ(), 2),
          new Point(0., 50.),
          Imgproc.FONT_HERSHEY_SIMPLEX, 1.,new Scalar(255., 0., 255.)
          );
 
        Imgproc.circle(mat, new Point(320., 240.),15, new Scalar(255., 0., 0.));


        // System.out.println("detection\n" + detection);
        /*detection
          DetectionResult
          [centerX=323.57451712821165, centerY=194.17724229537492,
          corners=[277.43115234375, 230.31558227539068, 356.44201660156244, 246.62757873535156, 366.3365173339844, 160.68710327148432, 299.2017211914063, 155.28285217285156],
          decisionMargin=146.9007,
          hamming=0,
          homography=[18.332487103585557, -38.36250042786162, 323.57451712821165, -5.76446208898901, 21.614215840129532, 194.17724229537492, -0.05518547782273199, -0.09321877962696339, 1.0],
          family=tag16h5,
          id=1]
        */
        // System.out.println("pose\n" + pose);
        /*
    good id 1 147.25876
    detection
    DetectionResult
    [centerX=323.9410157751192, centerY=193.97581883567503,
    corners=[277.6772766113281, 230.39865112304688, 357.2471313476563, 246.39450073242188, 366.3916320800782, 160.55500793457034, 299.3013610839844, 155.19680786132815],
    decisionMargin=147.25876,
    hamming=0,
    homography=[19.037271045344944, -39.16160043418358, 323.9410157751192, -5.476149885930315, 21.04368126256901, 193.97581883567503, -0.05328977565802691, -0.09627180307269138, 1.0],
    family=tag16h5,
    id=1]
    pose
    Transform3d
    (Translation3d(X: -0.04, Y: -0.02, Z: 1.25),
    Rotation3d(Quaternion(0.9653166966236213, -0.06736673085009076, 0.2387338188090715, 0.08143440643283699)))
        */

      }

      // put list of tags onto dashboard
      pubTags.set(tags.stream().mapToLong(Long::longValue).toArray());


      // Give the output stream a new image to display
      outputStream.putFrame(mat);
    }

    pubTags.close();
    detector.close();
  }

}
/*
decomposeHomographyMat
Decompose a homography matrix to rotation(s), translation(s) and plane normal(s).

C++: int decomposeHomographyMat(InputArray H, InputArray K, OutputArrayOfArrays rotations,
 OutputArrayOfArrays translations, OutputArrayOfArrays normals)

Parameters:	
H – The input homography matrix between two images.
K – The input intrinsic camera calibration matrix.
rotations – Array of rotation matrices.
translations – Array of translation matrices.
normals – Array of plane normal matrices.

This function extracts relative camera motion between two views observing a planar object
 from the homography H induced by the plane. The intrinsic camera matrix K must also be provided.
  The function may return up to four mathematical solution sets. At least two of the solutions
   may further be invalidated if point correspondences are available by applying positive depth
    constraint (all points must be in front of the camera). The decomposition method is
     described in detail in [Malis].

cameraMatrix – Output 3x3 floating-point camera matrix A = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1} .
fx 0 cx
0 fy cy
0  0  1

If CV_CALIB_USE_INTRINSIC_GUESS and/or CV_CALIB_FIX_ASPECT_RATIO are specified, some or all of fx, fy, cx, cy must be initialized before calling the function.
*/