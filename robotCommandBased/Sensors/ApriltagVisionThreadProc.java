package frc.robot.Sensors;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Spliterator;

import org.opencv.calib3d.Calib3d;
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
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ApriltagVisionThreadProc implements Runnable {
    
  {
    // prints the name of the Operating System
    System.out.println(System.getProperty("os.name"));
    /* maps a library name into a platform-specific string representing
       a native library */
    String str = System.mapLibraryName("os.name");   
    System.out.println(str);
    // File lib = null;
    // String os = System.getProperty("os.name");
    // String bitness = System.getProperty("sun.arch.data.model");
    // if (os.toUpperCase().contains("WINDOWS")) {
    //     if (bitness.endsWith("64")) {
    //         lib = new File("C:\\Users\\POWERUSER\\Downloads\\opencv\\build\\java\\x64\\"
    //                 + System.mapLibraryName("opencv_java2413"));
    //     } else {
    //         lib = new File("libs//x86//" + System.mapLibraryName("opencv_java2413"));
    //     }
    // }
    // System.load(lib.getAbsolutePath());
  }

public void run() {
    System.out.println("ApriltagVisionThreadProc");

    var detector = new AprilTagDetector();
    // look for tag16h5, don't correct any error bits
    detector.addFamily("tag16h5", 0);

  // Tag positions
  AprilTagFieldLayout aprilTagFieldLayout = null;

  try {
    aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
  } catch (IOException e) {
    e.printStackTrace();
  }

  System.out.println("Tags on file");
	Spliterator<AprilTag> namesSpliterator = aprilTagFieldLayout.getTags().spliterator();  // Getting Spliterator
	namesSpliterator.forEachRemaining(System.out::println);		// Traversing and printing elements
  /*
  Tags on file
AprilTag(ID: 1, pose: Pose3d(Translation3d(X: 15.51, Y: 1.07, Z: 0.46), Rotation3d(Quaternion(0.0, 0.0, 0.0, 1.0))))
AprilTag(ID: 2, pose: Pose3d(Translation3d(X: 15.51, Y: 2.75, Z: 0.46), Rotation3d(Quaternion(0.0, 0.0, 0.0, 1.0))))
AprilTag(ID: 3, pose: Pose3d(Translation3d(X: 15.51, Y: 4.42, Z: 0.46), Rotation3d(Quaternion(0.0, 0.0, 0.0, 1.0))))
AprilTag(ID: 4, pose: Pose3d(Translation3d(X: 16.18, Y: 6.75, Z: 0.70), Rotation3d(Quaternion(0.0, 0.0, 0.0, 1.0))))
AprilTag(ID: 5, pose: Pose3d(Translation3d(X: 0.36, Y: 6.75, Z: 0.70), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))
AprilTag(ID: 6, pose: Pose3d(Translation3d(X: 1.03, Y: 4.42, Z: 0.46), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))
AprilTag(ID: 7, pose: Pose3d(Translation3d(X: 1.03, Y: 2.75, Z: 0.46), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))
AprilTag(ID: 8, pose: Pose3d(Translation3d(X: 1.03, Y: 1.07, Z: 0.46), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))
 */

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

      for (AprilTagDetection detection : detections) { // loop all detections of AprilTags
        
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
        Transform3d pose = estimator.estimate(detection); // camera to object (I hope, maybe object to camera?)
        // System.out.println("camera to tag pose " + pose);

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
        
        Imgproc.rectangle(mat, new Point(0., 0.), new Point(640., 52.), new Scalar(255., 255., 255.), -1);

        Imgproc.putText(mat,
         String.format("pose (x, y, z meters) %,6.2f %,6.2f %,6.2f",
          pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getTranslation().getZ()),
          new Point(0., 25.),
          Imgproc.FONT_HERSHEY_SIMPLEX,0.6, new Scalar(200., 0., 255.));

        Imgproc.putText(mat,
          String.format("angle (Tx, Ty, Tz rads) %,6.2f %,6.2f %,6.2f",
            pose.getRotation().getX(), pose.getRotation().getY(), pose.getRotation().getZ()),
          new Point(0., 50.),
          Imgproc.FONT_HERSHEY_SIMPLEX, 0.6,new Scalar(200., 0., 255.)
          );
 
        Imgproc.circle(mat, new Point(320., 240.),15, new Scalar(255., 0., 0.));
        Imgproc.circle(mat, new Point(320., 240.),16, new Scalar(0., 255., 255.));

        // System.out.println("detection\n" + detection);
        // System.out.println("pose\n" + pose);

      var robotToCamera =
            new Transform3d(
              // new Translation3d(0.2, 0.0, 0.0), // camera in front of center of robot and at the bottom (low near ground)          
              new Translation3d(0.2, 0.0, 0.5), // camera in front of center of robot and above ground          
              new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0))); // camera in line with robot chassis

      // if(aprilTagFieldLayout.getTagPose(1).isPresent())
      // {
        var tagInField = aprilTagFieldLayout.getTagPose(1).get();
      // }

    // pose is camera to tag with its own co-ordinates that must be converted
      var poseFieldCoordinates = new Transform3d(tvecToTranslation(pose), rvecToRotation(pose));

      Pose3d robotInField = ComputerVisionUtil.objectToRobotPose(
        tagInField, // AprilTag Object in field
        poseFieldCoordinates,
        robotToCamera);
 
        tagsTable
        .getEntry("pose_" + detection.getId())
        .setDoubleArray(
            new double[] {
              robotInField.getTranslation().getX(), robotInField.getTranslation().getY(), robotInField.getTranslation().getZ(),
              robotInField.getRotation().getQuaternion().getX(), robotInField.getRotation().getQuaternion().getY(),
              robotInField.getRotation().getQuaternion().getZ(), robotInField.getRotation().getQuaternion().getW()
            });

        // System.out.println("robotInField " + robotInField);

      } // end loop over all detected tags

      // put list of tags onto dashboard
      pubTags.set(tags.stream().mapToLong(Long::longValue).toArray());

      // Give the output stream a new image to display
      outputStream.putFrame(mat);
 
    } // end while grab camera frame

    pubTags.close();
    detector.close();
  }
  
/* from PhotonVision - not sure I interpret it the same way - it's confusing
     * The AprilTag pose rotation outputs are X left, Y down, Z away from the tag
     * with the tag facing
     * the camera upright and the camera facing the target parallel to the floor.
     * But our OpenCV
     * solvePNP code would have X left, Y up, Z towards the camera with the target
     * facing the camera
     * and both parallel to the floor. So we apply a base rotation to the rotation
     * component of the
     * apriltag pose to make it consistent with the EDN system that OpenCV uses,
     * internally a 180
     * rotation about the X axis
     */

  /**
     * Returns a new 3d translation. The opencv tvec is a vector with three
     * elements representing {x, y, z} in the EDN coordinate system.
     *
     * @param tvecInput The tvec to create a Translation3d from
     */

  public static Translation3d tvecToTranslation(Transform3d tvecInput) {
    Translation3d tvecConverted = CoordinateSystem.convert(
      new Translation3d(tvecInput.getX(), tvecInput.getY(), -tvecInput.getZ()), // camera always facing tag (Z); flip with unary minus to get them in same direction
      CoordinateSystem.EDN(),
            CoordinateSystem.NWU());

    // System.out.println("tvec i " + tvecInput);
    // System.out.println("tvec o " + tvecConverted);
    // System.out.println(".");
    return tvecConverted;
}

    /**
     * Returns a 3d rotation. The opencv rvec is a vector with three
     * elements representing the axis scaled by the angle in the EDN coordinate system. (angle = norm,
     * and axis = rvec / norm)
     *
     * @param rvecInput The rvec to create a Rotation3d from
     */
  public static Rotation3d rvecToRotation(Transform3d rvecInput) {
    Vector<N3> axis = new Vector<>(Nat.N3());
    axis.set(0, 0, rvecInput.getRotation().getX());
    axis.set(1, 0, rvecInput.getRotation().getY());
    axis.set(2, 0, rvecInput.getRotation().getZ());
    var rvecConverted = rotationEDNtoNWU(new Rotation3d(axis.div(axis.norm()), axis.norm()));
    // System.out.println("rvec i " + rvecInput);
    // System.out.println("rvec o " + rvecConverted);
    // System.out.println(".");
    return rvecConverted;
}
  
    /**
     * Convert a rotation from EDN to NWU. For example, if you have a rotation X,Y,Z {1, 0, 0} in EDN,
     * this would be XYZ {0, -1, 0} in NWU.
     */
    private static Rotation3d rotationEDNtoNWU(Rotation3d rot) {
      return CoordinateSystem.convert(
                      new Rotation3d(), CoordinateSystem.NWU(), CoordinateSystem.EDN())
              .plus(CoordinateSystem.convert(rot, CoordinateSystem.EDN(), CoordinateSystem.NWU()));
  }
}
