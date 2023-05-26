//FIXME need to correctly handle Optional with no value present - it does happen sometimes

package frc.robot.Sensors;

import java.io.IOException;
import java.util.ArrayList;
import java.util.function.Consumer;

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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ApriltagVisionThreadProc implements Runnable {

  boolean ChargedUpTagLayout = false; // false is use custom deploy of layout

public void run() {
    System.out.println("ApriltagVisionThreadProc");

    var detector = new AprilTagDetector();
    // look for tag16h5, don't correct any error bits
    detector.addFamily("tag16h5", 0);

    // Tag positions
// tag rotation is CCW looking down on field from the ceiling.
// rotating around Z, 0 degrees is parallel to Y and facing down field or +X. 30 degrees is still
// facing down field +X and a little facing into the +Y across the field
    AprilTagFieldLayout aprilTagFieldLayout;
    try {
      if(ChargedUpTagLayout)
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile); // WPILib standard
      else
        aprilTagFieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2023-chargedup.json"); // my custom
      
    } catch (IOException e) {
      e.printStackTrace();
      aprilTagFieldLayout = null;
    }

    Consumer<AprilTag> printTag = tag ->
      {
        System.out.format("%s %6.1f, %6.1f, %6.1f [degrees]%n",
              tag.toString(),
              Units.radiansToDegrees(tag.pose.getRotation().getX()),
              Units.radiansToDegrees(tag.pose.getRotation().getY()),
              Units.radiansToDegrees(tag.pose.getRotation().getZ()));
      };

    System.out.println("Tags on file");    
    aprilTagFieldLayout.getTags().forEach(printTag);

    /*
Tags on file
AprilTag(ID: 1, pose: Pose3d(Translation3d(X: 10.51, Y: 8.00, Z: 0.46), Rotation3d(Quaternion(0.9658855493432025, 0.0, 0.0, 0.2589693139543369)))) 0.0, 0.0, 30.0 [degrees]
AprilTag(ID: 2, pose: Pose3d(Translation3d(X: 15.51, Y: 2.75, Z: 0.46), Rotation3d(Quaternion(0.0, 0.0, 0.0, 1.0)))) 0.0, 0.0, 180.0 [degrees]
AprilTag(ID: 3, pose: Pose3d(Translation3d(X: 15.51, Y: 4.42, Z: 0.46), Rotation3d(Quaternion(0.0, 0.0, 0.0, 1.0)))) 0.0, 0.0, 180.0 [degrees]
AprilTag(ID: 4, pose: Pose3d(Translation3d(X: 16.18, Y: 6.75, Z: 0.70), Rotation3d(Quaternion(0.0, 0.0, 0.0, 1.0)))) 0.0, 0.0, 180.0 [degrees]
AprilTag(ID: 5, pose: Pose3d(Translation3d(X: 0.36, Y: 6.75, Z: 0.70), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0)))) 0.0, 0.0, 0.0 [degrees]
AprilTag(ID: 6, pose: Pose3d(Translation3d(X: 1.03, Y: 4.42, Z: 0.46), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0)))) 0.0, 0.0, 0.0 [degrees]
AprilTag(ID: 7, pose: Pose3d(Translation3d(X: 1.03, Y: 2.75, Z: 0.46), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0)))) 0.0, 0.0, 0.0 [degrees]
AprilTag(ID: 8, pose: Pose3d(Translation3d(X: 1.03, Y: 1.07, Z: 0.46), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0)))) 0.0, 0.0, 0.0 [degrees]
  */

    // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
    // fx camera horizontal focal length, in pixels
    // fy camera vertical focal length, in pixels
    // cx camera horizontal focal center, in pixels
    // cy camera vertical focal center, in pixels
    double cameraFx = 699.377810315881;
    double cameraFy = 677.7161226393544;
    double cameraCx = 345.6059345433618;
    double cameraCy = 207.12741326228522;

    double tagSize = 0.1524; // meters

    // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
    var poseEstConfig =
        new AprilTagPoseEstimator.Config( tagSize, cameraFx, cameraFy, cameraCx, cameraCy);

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

        // loop all detections of AprilTags
        for (AprilTagDetection detection : detections) {
          
            if(detection.getId() > 8 || detection.getDecisionMargin() < 80.) // margin < 20 seems bad  > 140 are good maybe > 50 a limit?
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

        // determine pose transform from tag to camera
        // but later will have coordinate system changed and flip some angles and distance
        // for the final tag to camera transform that can be used.
        Transform3d pose = estimator.estimate(detection);

        { /* draw a 3-D box in front of the AprilTag */
        // Corner locations distorted by perspective found by AprilTag detector.
        // Couldn't get homography from that WPILib estimator to work so redoing estimate in OpenCV.
        // Reformating to use OpenCV solvePnP to get the homography that works
        // to draw the 3-D box in front of the tag.

        Mat H = new Mat(3, 3, CvType.CV_32F);
        H.put(0, 0, detection.getHomography());

        MatOfPoint2f scene = new MatOfPoint2f(
          new Point(detection.getCornerX(0), detection.getCornerY(0)),
          new Point(detection.getCornerX(1), detection.getCornerY(1)),
          new Point(detection.getCornerX(2), detection.getCornerY(2)),
          new Point(detection.getCornerX(3), detection.getCornerY(3))
          );

        // camera same as above but different format for OpenCV
        float[] cameraParm = {(float)cameraFx,   0.f,              (float)cameraCx,
                               0.f,               (float)cameraFy, (float)cameraCy,
                               0.f,                      0.f,                1.f};   
        Mat K = new Mat(3, 3, CvType.CV_32F); // camera matrix
        K.put(0, 0, cameraParm);

        MatOfDouble distCoeffs = new MatOfDouble(); // not using any camera distortions so it's empty

        // 3D points of ideal, original corners
        MatOfPoint3f bottom = new MatOfPoint3f(
          new Point3(-1.,1., 0.),
              new Point3(1., 1., 0.),
              new Point3(1., -1., 0.),
              new Point3(-1., -1., 0.));
       
        MatOfPoint3f top = new MatOfPoint3f(
          new Point3(-1.,1., -0.621),
              new Point3(1., 1., -0.621),
              new Point3(1., -1., -0.621),
              new Point3(-1., -1., -0.621));

        Mat R = new Mat();
        Mat T = new Mat();
        Calib3d.solvePnP(bottom, scene, K, distCoeffs, R, T);

        MatOfPoint2f imagePointsBottom = new MatOfPoint2f();
        Calib3d.projectPoints(bottom, R, T, K, distCoeffs, imagePointsBottom);

        MatOfPoint2f imagePointsTop = new MatOfPoint2f();

        Calib3d.projectPoints(top, R, T, K, distCoeffs, imagePointsTop); // good from solvePnP
        
        ArrayList<Point> topCornerPoints = new ArrayList<Point>();

        // draw from bottom points to top points - pillars
        for(int i = 0; i < 4; i++)
        {
            double x1;
            double y1;
            double x2;
            double y2;
            x1 = imagePointsBottom.get(i, 0)[0];
            y1 = imagePointsBottom.get(i, 0)[1];
            x2 = imagePointsTop.get(i, 0)[0];
            y2 = imagePointsTop.get(i, 0)[1];

            topCornerPoints.add(new Point(x2, y2));

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
        /* end draw a 3-D box in front of the AprilTag */
        }

        { /* put a circle in the center of the camera for aiming purposes */
        Imgproc.circle(mat, new Point(320., 240.),15, new Scalar(255., 0., 0.));
        Imgproc.circle(mat, new Point(320., 240.),16, new Scalar(0., 255., 255.));
        }

        // Transforms to get the robot pose in the field

        // Apriltag has known pose on the field
        //    loaded above from file
        // Detected tag's perspective seen by the camera is used to estimate the camera pose relative to the tag
        //    calculated above
        // Camera has a known pose relative to the robot chassis
        //    hard coded below
        // Combine this chain to find the robot pose in the field
        //    computed below

        /* Transforms, rotations, reflections that RKT doesn't understand but made the robot pose right.
           The coordinate system conversions were supposed to take care of all this.
           ->   .inverse    why is this needed in multiple places?
           ->   new Rotation3d(), CoordinateSystem.NWU(), CoordinateSystem.EDN()).plus(    why add desired pose to 0?
           ->   -tagToCameraTvec.getX()   why need flip upside down on the field?
           ->   -tagToCameraRvec.getZ()   why need reverse rotation?
        */

        var // transform from camera to robot chassis center which is located on the ground
        cameraToRobot = new Transform3d(
                             new Translation3d(-0.2, 0.0, -0.8), // camera in front of center of robot and above ground          
                            //  new Translation3d(0., 0., 0.), // camera at center of robot - good for testing other transforms          
                             new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0))); // camera in line with robot chassis

        //FIXME Fix all Optionals - getting no value present on next statement java.util.NoSuchElementException

        var // pose from WPILib
        tagInField = aprilTagFieldLayout.getTagPose(detection.getId()).get();

        var // tag to camera translation
        tagToCameraTvec = new Translation3d(pose.getZ(), -pose.getX(), pose.getY());

        var // tag to camera rotation
        tagToCameraRvec = new Rotation3d(-pose.getRotation().getZ(), pose.getRotation().getX(), Math.PI+pose.getRotation().getY());

        var // tag to camera transform
        tagToCamera = new Transform3d(tagToCameraTvec, tagToCameraRvec);

        var // robot in field is the composite of 3 pieces
        robotInField = tagInField // tag pose in field is known
                      .transformBy(tagToCamera) // vision gives transform from tag to camera for camera pose in field
                      .transformBy(cameraToRobot); // transform from camera to to robot is known for robot pose in field

        // end transforms to get the robot pose from this vision tag pose

        // robotInField = new Pose3d(); // zeros for test data

        // put out to SmartDashboard tag and if multiple tags they clash
        SmartDashboard.putNumber("robot Tx" + detection.getId(), robotInField.getX());
        SmartDashboard.putNumber("robot Ty" + detection.getId(), robotInField.getY());
        SmartDashboard.putNumber("robot Tz" + detection.getId(), robotInField.getZ());
        SmartDashboard.putNumber("robot Rx" + detection.getId(), Units.radiansToDegrees(robotInField.getRotation().getX()));
        SmartDashboard.putNumber("robot Ry" + detection.getId(), Units.radiansToDegrees(robotInField.getRotation().getY()));
        SmartDashboard.putNumber("robot Rz" + detection.getId(), Units.radiansToDegrees(robotInField.getRotation().getZ()));

        SmartDashboard.putNumber("tag to camera Tx" + detection.getId(), pose.getX());
        SmartDashboard.putNumber("tag to camera Ty" + detection.getId(), pose.getY());
        SmartDashboard.putNumber("tag to camera Tz" + detection.getId(), pose.getZ());
        SmartDashboard.putNumber("tag to camera Rx" + detection.getId(), Units.radiansToDegrees(pose.getRotation().getX()));
        SmartDashboard.putNumber("tag to camera Ry" + detection.getId(), Units.radiansToDegrees(pose.getRotation().getY()));
        SmartDashboard.putNumber("tag to camera Rz" + detection.getId(), Units.radiansToDegrees(pose.getRotation().getZ()));

        tagsTable
          .getEntry("robotpose_" + detection.getId())
          .setDoubleArray(
              new double[] {
                robotInField.getTranslation().getX(), robotInField.getTranslation().getY(), robotInField.getTranslation().getZ(),
                robotInField.getRotation().getQuaternion().getW(), robotInField.getRotation().getQuaternion().getX(),
                robotInField.getRotation().getQuaternion().getY(), robotInField.getRotation().getQuaternion().getZ()
              });
        
        // put out to NetworkTables this tag pose
        tagsTable
        .getEntry("tagpose_" + detection.getId())
        .setDoubleArray(
            new double[] {
                aprilTagFieldLayout.getTagPose(detection.getId()).get().getTranslation().getX(),
                aprilTagFieldLayout.getTagPose(detection.getId()).get().getTranslation().getY(),
                aprilTagFieldLayout.getTagPose(detection.getId()).get().getTranslation().getZ(),
                aprilTagFieldLayout.getTagPose(detection.getId()).get().getRotation().getQuaternion().getW(),
                aprilTagFieldLayout.getTagPose(detection.getId()).get().getRotation().getQuaternion().getX(),
                aprilTagFieldLayout.getTagPose(detection.getId()).get().getRotation().getQuaternion().getY(),
                aprilTagFieldLayout.getTagPose(detection.getId()).get().getRotation().getQuaternion().getZ()
            });
    
      } // end loop over all detected tags

      // put list of tags onto dashboard (NetworkTables)
      pubTags.set(tags.stream().mapToLong(Long::longValue).toArray());

      // Give the output stream a new image to display
      outputStream.putFrame(mat);
 
    } // end while grab camera frame

    pubTags.close();

    detector.close();
  }
}
// stuff from PhotonVision or others

// /* from PhotonVision
//      * The AprilTag pose rotation outputs are X left, Y down, Z away from the tag
//      * with the tag facing
//      * the camera upright and the camera facing the target parallel to the floor.
//      * But our OpenCV
//      * solvePNP code would have X left, Y up, Z towards the camera with the target
//      * facing the camera
//      * and both parallel to the floor. So we apply a base rotation to the rotation
//      * component of the
//      * apriltag pose to make it consistent with the EDN system that OpenCV uses,
//      * internally a 180
//      * rotation about the X axis
//      */

//     /**
//      * Returns a 3d rotation. The opencv rvec is a vector with three
//      * elements representing the axis scaled by the angle in the EDN coordinate system. (angle = norm,
//      * and axis = rvec / norm)
//      *
//      * @param rvecInput The rvec to create a Rotation3d from
//      */
//   public static Rotation3d rvecToRotation(Transform3d rvecInput) {
//     Vector<N3> axis = new Vector<>(Nat.N3());
//     axis.set(0, 0, rvecInput.getRotation().getX());
//     axis.set(1, 0, rvecInput.getRotation().getY());
//     axis.set(2, 0, rvecInput.getRotation().getZ());
//     var rvecConverted = rotationEDNtoNWU(new Rotation3d(axis.div(axis.norm()), axis.norm()));
//     System.out.println("rvec i " + rvecInput);
//     System.out.println("rvec o " + rvecConverted);
//     System.out.println(".");
//     return rvecConverted;
// }
  
//     /**
//      * Convert a rotation from EDN to NWU. For example, if you have a rotation X,Y,Z {1, 0, 0} in EDN,
//      * this would be XYZ {0, -1, 0} in NWU.
//      */
//     private static Rotation3d rotationEDNtoNWU(Rotation3d rot) {
//       return CoordinateSystem.convert(
//                       new Rotation3d(), CoordinateSystem.NWU(), CoordinateSystem.EDN())
//               .plus(CoordinateSystem.convert(rot, CoordinateSystem.EDN(), CoordinateSystem.NWU()));
//   }

/*
https://github.com/4201VitruvianBots/ChargedUp2023AprilTags/blob/c9830543d2c972bcd088209a7bcfd3ba93068d7a/tests/testTagToCameraPose.py

    wpiTranslation = CoordinateSystem.convert(tagTransform.translation().rotateBy(tagTransform.inverse().rotation()),
                                              CoordinateSystem.EDN(),
                                              CoordinateSystem.NWU())
    # wpiTranslation = wpiTranslation.rotateBy(tagTransform.rotation())
    robotPose = tagPose.transformBy(Transform3d(wpiTranslation, Rotation3d())).transformBy(robotToCamera)
 */
/*
 from WPILib documentation Drive classes:
 Axis Conventions:
The drive classes use the NWU axes convention (North-West-Up as external reference in the world frame).
 The positive X axis points ahead, the positive Y axis points left, and the positive Z axis points up.
  We use NWU here because the rest of the library, and math in general, use NWU axes convention.

Joysticks follow NED (North-East-Down) convention, where the positive X axis points ahead, the
 positive Y axis points right, and the positive Z axis points down. However, it’s important to note
  that axes values are rotations around the respective axes, not translations. When viewed with each
   axis pointing toward you, CCW is a positive value and CW is a negative value. Pushing forward on the joystick is a CW rotation around the Y axis, so you get a negative value. Pushing to the right is a CCW rotation around the X axis, so you get a positive value.
 */