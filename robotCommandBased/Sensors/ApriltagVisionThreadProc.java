//FIXME need to correctly handle Optional with no value present - it does happen sometimes

package frc.robot.Sensors;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
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
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;

public class ApriltagVisionThreadProc implements Runnable {
 
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
      aprilTagFieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2023-chargedup.json"); // my custom
      // aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile); // WPILib standard
    } catch (IOException e) {
      e.printStackTrace();
      aprilTagFieldLayout = null;
    }

    Consumer<AprilTag> printTag = (tag) ->
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

  List<AprilTag> modifiedTags = aprilTagFieldLayout.getTags(); // make my copy of the tags so I can change them
  
  aprilTagFieldLayout = new AprilTagFieldLayout(modifiedTags, 16.54, 8.);

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

        // determine pose
        Transform3d pose = estimator.estimate(detection); // camera to object (I hope, maybe object to camera?)

        /* draw a 3-D box in front of the AprilTag */

        Mat H = new Mat(3, 3, CvType.CV_32F);
        H.put(0, 0, detection.getHomography());

        // original, before distortion corner locations
        MatOfPoint2f obj = new MatOfPoint2f(new Point(-1.,1.), new Point(1., 1.), new Point(1., -1.), new Point(-1., -1.));

        // distorted by perspective corner locations found by AprilTag detector
        // reformating to use in OpenCV solvePnP to get the homography that works
        // to draw the 3-D box in front of the tag
        MatOfPoint2f scene = new MatOfPoint2f(
          new Point(detection.getCornerX(0), detection.getCornerY(0)),
          new Point(detection.getCornerX(1), detection.getCornerY(1)),
          new Point(detection.getCornerX(2), detection.getCornerY(2)),
          new Point(detection.getCornerX(3), detection.getCornerY(3))
          );

        // camera same as above but different format for OpenCV
        float[] cameraParm = {(float)cameraConfig[0],   0.f,              (float)cameraConfig[2],
                               0.f,               (float)cameraConfig[1], (float)cameraConfig[3],
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
            double x1, y1, x2, y2;
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

        /* display camera to tag transform at the top of the image and put acircle in the center*/
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
        /* end display camera to tag transform at the top of the image and put acircle in the center*/

        // Transforms to get the robot pose

        // Apriltag has known pose on the field
        //    loaded above from file
        // Detected tag's perspective seen by the camera is used to estimate the camera pose relative to the tag
        //    calculated above
        // Camera has a known pose relative to the robot chassis
        //   hard coded below
        // Combine this chain to find the robot pose in the field
        //    computed below

        /* Transforms, rotations, reflections that I do not understand.
           I thought the coordinate system conversions were supposed to take care of all this.
           I fear these patches are brittle and the scheme will blow up when tags are located
           at other angles to the field (to be tested).
           ->   .inverse    why is this needed in multiple places?
           ->   new Rotation3d(), CoordinateSystem.NWU(), CoordinateSystem.EDN()).plus(    why add desired pose to 0?
           ->   -tagToCameraTvec.getZ()   why need flip upside down on the field?
           ->   -tagToCameraRvec.getX()   why need reverse rotation?
           ->   -tagToCameraRvec.getY()   why need reverse rotation?
        */

        var // transform from camera to robot chassis center which is located on the ground
        cameraToRobot = new Transform3d(
                             new Translation3d(-0.2, 0.0, -0.8), // camera in front of center of robot and above ground          
                               new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0))); // camera in line with robot chassis

        //FIXME getting no value present on next statement java.util.NoSuchElementException
        var // pose from WPILib
        tagInField = aprilTagFieldLayout.getTagPose(detection.getId()).get();

        var // tag to camera translation
        tagToCameraTvec = CoordinateSystem.convert(pose.getTranslation()
                                                              .rotateBy(pose.inverse().getRotation()),
                                                  CoordinateSystem.EDN(),CoordinateSystem.NWU()); // up/down is upside down
        tagToCameraTvec = new Translation3d(
                            tagToCameraTvec.getX(), // this one is okay
                            tagToCameraTvec.getY(), // this one is okay
                            -tagToCameraTvec.getZ()); // PATCH translation reversed

        var // tag to camera rotation
        tagToCameraRvec = CoordinateSystem.convert(new Rotation3d(), CoordinateSystem.NWU(), CoordinateSystem.EDN())
                                          .plus(CoordinateSystem.convert(pose.inverse().getRotation(),
                                                                        CoordinateSystem.EDN(), CoordinateSystem.NWU()));

        tagToCameraRvec = new Rotation3d(
                          -tagToCameraRvec.getX(), // PATCH rotation reversed             
                          -tagToCameraRvec.getY(), // PATCH rotation reversed             
                           tagToCameraRvec.getZ() ); // this one is okay

        var // tag to camera transform
        tagToCamera = new Transform3d(tagToCameraTvec, tagToCameraRvec);

        var // robot in field is the composite of 3 pieces
        robotInField = tagInField // tag pose in field is known
                      .transformBy(tagToCamera) // vision gives transform from tag to camera for camera pose in field
                      .transformBy(cameraToRobot); // transform from camera to to robot is known for robot pose in field

        // display the robot pose to field
        Imgproc.rectangle(mat, new Point(0., 53.), new Point(640., 104.), new Scalar(255., 255., 255.), -1);

        Imgproc.putText(mat,
          String.format("pose (x, y, z meters) %,6.2f %,6.2f %,6.2f",
          robotInField.getX(), robotInField.getY(), robotInField.getZ()),
            new Point(0., 75.),
            Imgproc.FONT_HERSHEY_SIMPLEX,0.6, new Scalar(200., 0., 255.));

        Imgproc.putText(mat,
            String.format("angle (Tx, Ty, Tz rads) %,6.2f %,6.2f %,6.2f",
            robotInField.getRotation().getX(), robotInField.getRotation().getY(), robotInField.getRotation().getZ()),
            new Point(0., 100.),
            Imgproc.FONT_HERSHEY_SIMPLEX, 0.6,new Scalar(200., 0., 255.)
          );
  
        // end transforms to get the robot pose from this vision tag pose

        // robotInField = new Pose3d(); // zeros for test data

        // put out to NetworkTables the robot pose according to this tag
        tagsTable
          .getEntry("robotpose_" + detection.getId())
          .setDoubleArray(
              new double[] {
                robotInField.getTranslation().getX(), robotInField.getTranslation().getY(), robotInField.getTranslation().getZ(),
                robotInField.getRotation().getQuaternion().getW(), robotInField.getRotation().getQuaternion().getX(),
                robotInField.getRotation().getQuaternion().getY(), robotInField.getRotation().getQuaternion().getZ()
              });
        
        // put out to NetworkTables the tag pose
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

import math

from wpimath import geometry
from wpimath.geometry import CoordinateAxis, CoordinateSystem, Transform3d, Translation3d, Rotation3d, Quaternion, \
    Pose3d

from aprilTags.tag_dictionary import TAG_DICTIONARY


tag_dictionary = TAG_DICTIONARY

tagsToTest = [1, 2]
detectionResult = [
    Transform3d(Translation3d(-0.47, 0.04, 2.33), Rotation3d(math.radians(1.35), math.radians(22.20), math.radians(0.90))),
    Transform3d(Translation3d(-2.03, 0.00, 2.90), Rotation3d(math.radians(0.48), math.radians(19.35), math.radians(0.84))),
]


# RobotPose = (12.192, 0, 45)
robotToCamera = Transform3d(Translation3d(), Rotation3d(0, 0, 0))
for tag in tagsToTest:
    tagValues = tag_dictionary['tags'][tag - 1]['pose']
    tagTranslation = tagValues['translation']
    tagRotation = tagValues['rotation']['quaternion']
    tagT3D = Translation3d(tagTranslation['x'], tagTranslation['y'], tagTranslation['z']) # position of the tag on the field
    tagR3D = Rotation3d(Quaternion(tagRotation['W'], tagRotation['X'], tagRotation['Y'], tagRotation['Z'])) # rotation of the tag on the field
    tagPose = Pose3d(tagT3D, tagR3D) # pose of the tag on the field

    tagTransform = detectionResult[tag - 1] # camera to tag, I think
    wpiTranslation = CoordinateSystem.convert(tagTransform.translation().rotateBy(tagTransform.inverse().rotation()),
                                              CoordinateSystem.EDN(),
                                              CoordinateSystem.NWU())
    # wpiTranslation = wpiTranslation.rotateBy(tagTransform.rotation())
    robotPose = tagPose.transformBy(Transform3d(wpiTranslation, Rotation3d())).transformBy(robotToCamera)

    print("Tag Position - X: {:.02f}\tY: {:.02f}\tZ: {:.02f}\tR: {:.02f}".format(tagPose.x, tagPose.y, tagPose.z, tagPose.rotation().z_degrees))
    print("Tag Translation: {}".format(wpiTranslation))
    print("robotpose: ({:.02f}, {:.02f}, {:.02f})".format(robotPose.translation().x, robotPose.translation().y, robotPose.rotation().y_degrees))

# tagTransform2 = Transform3d(Translation3d(-0.78, 0, 1.82), Rotation3d(math.radians(-180), math.radians(45), math.radians(0)))
# wpiTranslation2 = CoordinateSystem.convert(tagTransform2.translation(),
#                                           CoordinateSystem.EDN(),
#                                           CoordinateSystem.NWU())
#
# robotPose2 = tagPose.transformBy(Transform3d(wpiTranslation2, tagTransform2.rotation())) \
#                     .transformBy(robotToCamera)
#
# print("Tag Translation2: {}".format(wpiTranslation))
# print("robotpose2: ({:.02f}, {:.02f}, {:.02f})".format(robotPose2.translation().x, robotPose2.translation().y, robotPose2.rotation().y_degrees))
 */