/*      Apriltag has known pose on the field
           loaded from file
        Detected tag's perspective seen by the camera is used to estimate the camera pose relative to the tag
           calculated
        Camera has a known pose relative to the robot chassis
           hard coded
        Combine this chain to find the robot pose in the field
           computed
*/
package frc.robot.Sensors;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Collectors;

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
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;

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

    System.out.println(aprilTagFieldLayout.getTags().size() + " Tags on file");    
    aprilTagFieldLayout.getTags().forEach(printTag);

    /*
8 Tags on file
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

            Pose3d tagInFieldFrame; // pose from WPILib or custom pose file

            if(aprilTagFieldLayout.getTagPose(detection.getId()).isPresent() && detection.getDecisionMargin() > 50.) // margin < 20 seems bad  > 140 are good maybe > 50 a limit?
            {
              tagInFieldFrame = aprilTagFieldLayout.getTagPose(detection.getId()).get();
            }
            else
            {
              System.out.println("bad id " + detection.getId() + " " + detection.getDecisionMargin() + " " + detection.getHamming());
              continue;
            }
    
          System.out.println("good id " + detection.getId() + " " + detection.getDecisionMargin() + " " + detection.getHamming());

          // remember we saw this tag
          tags.add((long) detection.getId());

        // determine pose transform from camera to tag
        // will have coordinate system changed for the final camera to tag then tag to camera transform that can be used.
        Transform3d tagInCameraFrame = estimator.estimate(detection);
        PrintPose.print("tag in camera frame EDN", detection.getId(), tagInCameraFrame);

        // Transforms to get the robot pose in the field
        // to match LimeLight Vision botpose_wpiblue network tables entries
        // as they display in AdvantageScope 3D Field

        // OpenCV and WPILib estimator layout of axes is EDN and field WPILib is NWU; need x -> -y , y -> -z , z -> x
        //tagInCameraFrame = CoordinateSystem.convert(tagInCameraFrame, CoordinateSystem.EDN(), CoordinateSystem.NWU()); // WPILib convert is wrong for transforms (2023.4.3)
        tagInCameraFrame = CoordinateSystemDOTconvert(tagInCameraFrame, CoordinateSystem.EDN(), CoordinateSystem.NWU()); // corrected convert
        PrintPose.print("tag in camera frame NWU", detection.getId(), tagInCameraFrame);

        var // transform to camera from robot chassis center which is located on the ground
        cameraInRobotFrame = new Transform3d(       
                             new Translation3d(0.2, 0., 0.8),// camera in front of center of robot and above ground
                             new Rotation3d(0.0, Units.degreesToRadians(-30.), Units.degreesToRadians(0.0))); // camera in line with robot chassis
                                                                                  // -30 camera points up; +30 points down -- backwards of LL
        // mysterious patches are needed make it work; why 2 extra rotations and offsets?
        tagInCameraFrame = new Transform3d(
                              new Translation3d(
                                              tagInCameraFrame.getX(),
                                              tagInCameraFrame.getY(),
                                              tagInCameraFrame.getZ()),
                                new Rotation3d(
                                            Math.PI-tagInCameraFrame.getRotation().getX(),
                                           Math.PI+tagInCameraFrame.getRotation().getY(),
                                            tagInCameraFrame.getRotation().getZ()));
        
        var // robot in field is the composite of 3 pieces
        robotInFieldFrame = ComputerVisionUtil.objectToRobotPose(tagInFieldFrame,  tagInCameraFrame,  cameraInRobotFrame);
        // end transforms to get the robot pose from this vision tag pose

        // robotInField = new Pose3d(); // zeros for test data
        
        PrintPose.print("tag in field frame", detection.getId(),tagInFieldFrame);
        PrintPose.print("camera in robot frame", detection.getId(), cameraInRobotFrame);
        PrintPose.print("robot in field frame", detection.getId(), robotInFieldFrame);
    
        // put out to NetworkTables robot pose for this tag AdvantageScope format
        tagsTable
          .getEntry("robotpose_" + detection.getId())
          .setDoubleArray(
              new double[] {
                      robotInFieldFrame.getTranslation().getX(),
                      robotInFieldFrame.getTranslation().getY(),
                      robotInFieldFrame.getTranslation().getZ(),
                      robotInFieldFrame.getRotation().getQuaternion().getW(),
                      robotInFieldFrame.getRotation().getQuaternion().getX(),
                      robotInFieldFrame.getRotation().getQuaternion().getY(),
                      robotInFieldFrame.getRotation().getQuaternion().getZ()
              });
        
        // put out to NetworkTables this tag pose AdvantageScope format
        tagsTable
          .getEntry("tagpose_" + detection.getId())
          .setDoubleArray(
              new double[] {
                      tagInFieldFrame.getTranslation().getX(),
                      tagInFieldFrame.getTranslation().getY(),
                      tagInFieldFrame.getTranslation().getZ(),
                      tagInFieldFrame.getRotation().getQuaternion().getW(),
                      tagInFieldFrame.getRotation().getQuaternion().getX(),
                      tagInFieldFrame.getRotation().getQuaternion().getY(),
                      tagInFieldFrame.getRotation().getQuaternion().getZ()
              });
    
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
        
        { /* draw a 3-D box in front of the AprilTag */
          // Corner locations distorted by perspective found by AprilTag detector.
          // WPILib estimator's R and T from a reversal of the WPILib homography
          // don't provide good numbers draw nice 3-D box. There are up to 4 solutions
          // and they aren't scaled or translated correctly
          // so redoing estimate with OpenCV solvePNP. The R and T are very similar
          // but behave dramatically differently unless the correct one is selected and massaged.

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

          // 3D points of ideal, original corners, flat on the tag, scaled to the actual tag size
          MatOfPoint3f bottom = new MatOfPoint3f(
            new Point3(-1.*tagSize/2.,1.*tagSize/2., 0.),
                new Point3(1.*tagSize/2., 1.*tagSize/2., 0.),
                new Point3(1.*tagSize/2., -1.*tagSize/2., 0.),
                new Point3(-1.*tagSize/2., -1.*tagSize/2., 0.));
          
          // 3D points of the ideal, original corners, above the tag to make a box, scaled to the actual tag size
          MatOfPoint3f top = new MatOfPoint3f(
            new Point3(-1.*tagSize/2.,1.*tagSize/2., -0.7*tagSize/2.),
                new Point3(1.*tagSize/2., 1.*tagSize/2., -0.7*tagSize/2.),
                new Point3(1.*tagSize/2., -1.*tagSize/2., -0.7*tagSize/2.),
                new Point3(-1.*tagSize/2., -1.*tagSize/2., -0.7*tagSize/2.));

          Mat R = new Mat();
          Mat T = new Mat();

          var
          foundSolution = Calib3d.solvePnP(bottom, scene, K, distCoeffs, R, T, false, Calib3d.SOLVEPNP_IPPE_SQUARE); // similar to WPILib
          if(!foundSolution)
          {
            System.out.println("no solvePnP solution");
            continue;
          }

          MatOfPoint2f imagePointsBottom = new MatOfPoint2f();
          Calib3d.projectPoints(bottom, R, T, K, distCoeffs, imagePointsBottom);

          MatOfPoint2f imagePointsTop = new MatOfPoint2f();

          Calib3d.projectPoints(top, R, T, K, distCoeffs, imagePointsTop);
          
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
        } /* end draw a 3-D box in front of the AprilTag */

      } // end loop over all detected tags

      { /* put a circle in the center of the camera image for aiming purposes */
        Imgproc.circle(mat, new Point(320., 240.),15, new Scalar(255., 0., 0.));
        Imgproc.circle(mat, new Point(320., 240.),16, new Scalar(0., 255., 255.));
      }

      // put list of tags onto dashboard (NetworkTables)
      pubTags.set(tags.stream().mapToLong(Long::longValue).toArray());

      // Give the output stream a new image to display
      outputStream.putFrame(mat);
 
    } // end while grab camera frame

    pubTags.close();

    detector.close();
  } // end method run

  /*
   * Patch WPILib CoordinateSystem.convert(Transform3d transform, CoordinateSystem from, coordinateSystem to)
   */
  public static Transform3d CoordinateSystemDOTconvert(
      Transform3d transform, CoordinateSystem from, CoordinateSystem to) {   
    return new Transform3d(
CoordinateSystem.convert(transform.getTranslation(), from, to),
  CoordinateSystem.convert(new Rotation3d(), to, from)
                        .plus(CoordinateSystem.convert(transform.getRotation(), from, to)));
  } // end method CoordinateSystemDOTconvert

  
  String toStringDA(double[] array) {
    return Arrays.stream(array)
            .mapToObj(i -> String.format("%5.2f", i))
           // .collect(Collectors.joining(", ", "[", "]"));
            .collect(Collectors.joining("|", "|", "|"));
  }
  
} // end class ApriltagVisionThreadProc

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

/*
https://github.com/4201VitruvianBots/ChargedUp2023AprilTags/blob/c9830543d2c972bcd088209a7bcfd3ba93068d7a/tests/testTagToCameraPose.py#L22-L34
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
    tagT3D = Translation3d(tagTranslation['x'], tagTranslation['y'], tagTranslation['z'])
    tagR3D = Rotation3d(Quaternion(tagRotation['W'], tagRotation['X'], tagRotation['Y'], tagRotation['Z']))
    tagPose = Pose3d(tagT3D, tagR3D)

    tagTransform = detectionResult[tag - 1]
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
/////////////////////////////////////////////
# Position of the Camera relative to the robot's center. For simplicity, assume it is the same.
robotToCamera = Transform3d(Translation3d(), Rotation3d(0, 0, 0))

# Using Tag 1 Position
tagT3D = geometry.Translation3d(15.51, 2.75, 0.46)
tagR3D = geometry.Rotation3d(geometry.Quaternion(0, 0, 0, 1))
tagPose3D = geometry.Pose3d(tagT3D, tagR3D)

# Assume the tag is in front of you with no rotation
tagToCameraTransform = geometry.Transform3d(geometry.Translation3d(-3, -2, -1), geometry.Rotation3d(0, 0, 0))

wpiTranslation = CoordinateSystem.convert(tagToCameraTransform.translation().rotateBy(tagToCameraTransform.inverse().rotation()),
                                          CoordinateSystem.EDN(),
                                          CoordinateSystem.NWU())

robotPose = tagPose.transformBy(Transform3d(wpiTranslation, Rotation3d())).transformBy(robotToCamera)
*/

/*
 * // Mat rotMat = new Mat(3, 3, MatType.CV_64FC1);
//  Mat rotMat = new Mat();;
// Calib3d.Rodrigues(R, rotMat);
// System.out.println("Rodrigues\n" + rotMat.dump());
/*
cv::SOLVEPNP_IPPE_SQUARE Method is based on the paper of Toby Collins and Adrien Bartoli. "Infinitesimal Plane-Based Pose Estimation" ([51]). This method is suitable for marker pose estimation. It requires 4 coplanar object points defined in the following order:
point 0: [-squareLength / 2, squareLength / 2, 0]
point 1: [ squareLength / 2, squareLength / 2, 0]
point 2: [ squareLength / 2, -squareLength / 2, 0]
point 3: [-squareLength / 2, -squareLength / 2, 0]

Rodrigues
[0.9769217768652474, -0.04939461269882756, -0.2078076373061888;
0.1403418250215021, 0.8818570093966047, 0.4501470727747885;
0.1610217812473884, -0.4689225812587932, 0.868437446653188]
 */

 /*
Mat Rvec = new Mat();
Mat Tvec = new Mat();
Mat rvec = new Mat(1, 3, MatType.CV_64FC1);
Mat tvec = new Mat(1, 3, MatType.CV_64FC1);
 
Cv2.SolvePnP(
    op,
    ip,
    camMat,
    distCoeffs,
    rvec,
    tvec
);
 
rvec.ConvertTo(Rvec, MatType.CV_32F);
tvec.ConvertTo(Tvec, MatType.CV_32F);
 
Mat rotMat = new Mat(3, 3, MatType.CV_64FC1);
 
Cv2.Rodrigues(Rvec, rotMat);
  */

    
// // demo of WPILib bug report
// private void testit()
// {
//   for(int axis = 0; axis<=2; axis++)
// {
//   double[] v = {0., 0., 0.};
//   v[axis] = 10.;
//   var tp = new Transform3d(
//                 new Translation3d(v[0], v[1], v[2]),
//                 new Rotation3d(
//                     Units.degreesToRadians(v[0]),
//                    Units.degreesToRadians(v[1]),
//                     Units.degreesToRadians(v[2])));

//   System.out.println("\ntp\n"
//     + tp.getX() + " " + tp.getY() + " " + tp.getZ()
//     + "; " + Units.radiansToDegrees(tp.getRotation().getX())
//     + " " + Units.radiansToDegrees(tp.getRotation().getY())
//     + " " + Units.radiansToDegrees(tp.getRotation().getZ())
//     );

//   var tpc = CoordinateSystemDOTconvert(tp, CoordinateSystem.EDN(), CoordinateSystem.NWU());         

//   System.out.println("tp converted\n"
//     + tpc.getX() + " " + tpc.getY() + " " + tpc.getZ()
//     + "; " + Units.radiansToDegrees(tpc.getRotation().getX())
//     + " " + Units.radiansToDegrees(tpc.getRotation().getY())
//     + " " + Units.radiansToDegrees(tpc.getRotation().getZ())
//     );
// }
// // var origin = CoordinateSystem.convert(new Pose3d(), CoordinateSystem.EDN(), CoordinateSystem.NWU());

// // System.out.println("\norigin pose3d converted\n"
// //   + origin.getX() + " " + origin.getY() + " " + origin.getZ()
// //   + "; " + Units.radiansToDegrees(origin.getRotation().getX())
// //   + " " + Units.radiansToDegrees(origin.getRotation().getY())
// //   + " " + Units.radiansToDegrees(origin.getRotation().getZ())
// //   );

// var noChange = CoordinateSystemDOTconvert(new Transform3d(), CoordinateSystem.EDN(), CoordinateSystem.NWU());

// System.out.println("no change transform3d converted\n"
//   + noChange.getX() + " " + noChange.getY() + " " + noChange.getZ()
//   + "; " + Units.radiansToDegrees(noChange.getRotation().getX())
//   + " " + Units.radiansToDegrees(noChange.getRotation().getY())
//   + " " + Units.radiansToDegrees(noChange.getRotation().getZ())
//   );
// }
// // /*
// // tp
// // 10.0 0.0 0.0; 10.0 0.0 0.0
// // tp converted
// // 0.0 -10.0 0.0; -80.00000000000001 0.0 -90.0

// // tp
// // 0.0 10.0 0.0; 0.0 10.0 0.0
// // tp converted
// // 0.0 0.0 -10.0; -90.0 0.0 -100.0

// // tp
// // 0.0 0.0 10.0; 0.0 0.0 10.0
// // tp converted
// // 10.0 0.0 0.0; -90.0 9.999999999999998 -90.0

// // origin pose3d converted
// // 0.0 0.0 0.0; -90.0 0.0 -90.0
// // no change transform3d converted
// // 0.0 0.0 0.0; -90.0 0.0 -90.0
// //  */
// // END demo of WPILib bug report


//////////////////////////////////////
          // List<Mat> RMatrixlist = new ArrayList<Mat>();
          // List<Mat> Tvectorlist = new ArrayList<Mat>();

//           double[] testgethomo = detection.getHomography();
//           System.out.println(toStringDA(testgethomo));

//           Mat testit = new Mat(3, 3, CvType.CV_32F);
//           testit.put(0, 0, 1.,2.,3.,4.,5.,6.,7.,8.,9.); // row major input order
//           System.out.println(testit.dump());
// /*
// [1, 2, 3;
//   4, 5, 6;
//   7, 8, 9]
// */
          // List<Mat> normal = new ArrayList<Mat>();
          // Mat H = new Mat(3, 3, CvType.CV_64F);
          // H.put(0, 0, detection.getHomography());
          // Calib3d.decomposeHomographyMat(H, K, RMatrixlist, Tvectorlist, normal);
          // System.out.println(Rlist);
          // System.out.println(Tlist);
          // System.out.println(normal);

          // for(int i = 0; i < Rlist.size(); i++)
          // {
          //   System.out.println(
          //     "\nR " + i + "\n" + Rlist.get(i).dump() +
          //     "\nT " + i + "\n" + Tlist.get(i).dump() +
          //     "\nnormal " + i + "\n" + normal.get(i).dump()
          //   );
          // }
//[Mat [ 3*3*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x13b7e18, dataAddr=0x13b8080 ], Mat [ 3*3*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x13b7e58, dataAddr=0x13b8100 ], Mat [ 3*3*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x13b7e98, dataAddr=0x13b81c0 ], Mat [ 3*3*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x13b7ed8, dataAddr=0x13b8280 ]]
//[Mat [ 3*1*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x13b8a78, dataAddr=0x13b8440 ], Mat [ 3*1*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x13b8ab8, dataAddr=0x13b8500 ], Mat [ 3*1*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x13b8af8, dataAddr=0x13b8580 ], Mat [ 3*1*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x13b8b38, dataAddr=0x13b8640 ]]
//[Mat [ 3*1*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x13b8c20, dataAddr=0x13b87c0 ], Mat [ 3*1*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x13b8c60, dataAddr=0x13b8840 ], Mat [ 3*1*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x13b8ca0, dataAddr=0x13b8900 ], Mat [ 3*1*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x13b8ce0, dataAddr=0x13b8980 ]]

          // Mat Rvector = new Mat(); 
          // Mat Tvector = new Mat(); 

          // System.out.println("R " + Rmatrix.dump());
          // System.out.println("T " + T.dump());

          // Mat rotMat = new Mat(3, 3, MatType.CV_64FC1);

          // Mat Rvec = new Mat();
          // Calib3d.Rodrigues(RMatrixlist.get(2), Rvec);



          // Rvec.copyTo(R);
          // Tvectorlist.get(2).copyTo(T);

          // System.out.println("Rs " + R.dump());
          // System.out.println("Ts " + T.dump());
/*
R [-0.3464558719874254;
0.5089872964519015;
-0.01670075000555101]

T [0.01681757319956745;
0.0002821889450013376;
0.9202925224984505]

Rs [0.04946394466164961, -0.4626563167101914, 0.8851567379771961;
-0.4633244730066544, 0.7744848669026645, 0.4307013160553419;
-0.8848071828884798, -0.4314189652145561, -0.1760509175255702]

Ts [1.044554312871123;
0.4409966185508314;
0.3912485505023551]
*/
          // // System.out.println("R\n" + R.dump() + "\n" + R.toString());
          // // System.out.println("T\n" + T.dump() + "\n" + T.toString());
          // double[] Ro = new double[3];
          // double[] To = new double[3];
          // Mat R = new Mat(3, 1, CvType.CV_64FC1);
          // Mat T = new Mat(3, 1, CvType.CV_64FC1);
          // R.put(0, 0, pose.getRotation().getX(), pose.getRotation().getY(), pose.getRotation().getZ());
          // T.put(0, 0, pose.getX(), pose.getY(), pose.getZ());
          // R.get(0, 0, Ro);
          // T.get(0, 0, To);
          // var
          // poseO = new Transform3d(new Translation3d(To[0],To[1],To[2]), new Rotation3d(Ro[0],Ro[1],Ro[2])); // easier to print
          // PrintPose.print("tag to camera frame SolvePnP", detection.getId(), poseO);
////////////////////////////////////////////////////////
