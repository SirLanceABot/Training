// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.TimedRobot;

import java.io.IOException;
import java.util.ArrayList;

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

/**
 * This is a demo program showing the detection of AprilTags. The image is acquired from the USB
 * camera, then any detected AprilTags are marked up on the image, transformed to the robot on
 * the field pose and sent to the dashboard.
 *
 * <p>Be aware that the performance on this is much worse than a coprocessor solution!
 */
public class Robot extends TimedRobot {
  @Override
  public void robotInit() {
    var visionThread = new Thread(() -> apriltagVisionThreadProc());
    visionThread.setDaemon(true);
    visionThread.start();
  }

  void apriltagVisionThreadProc() {
    var detector = new AprilTagDetector();
    // look for tag16h5, don't correct any error bits
    detector.addFamily("tag16h5", 0);

    // Positions of AprilTags
    AprilTagFieldLayout aprilTagFieldLayout;
    try {
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
      aprilTagFieldLayout = null;
    }

    double scaleResolutionWX = 0.5; // basis of camera width resolution is 640 so scaling to 320
    double scaleResolutionHY = 0.5; // basis of camera height resolution is 480 so scaling to 240
    // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
    // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)

    // fx camera horizontal focal length, in pixels
    // fy camera vertical focal length, in pixels
    // cx camera horizontal focal center, in pixels
    // cy camera vertical focal center, in pixels
    double cameraFx = 699.377810315881*scaleResolutionWX; // no scaling at 640
    double cameraFy = 677.7161226393544*scaleResolutionHY; // no scaling  at 480
    double cameraCx = 345.6059345433618*scaleResolutionWX; // no scaling  at 640
    double cameraCy = 207.12741326228522*scaleResolutionHY; // no scaling  at 480

    int cameraW = (int)(640*scaleResolutionWX); // no scaling at 640
    int cameraH = (int)(480*scaleResolutionHY); // no scaling  at 480

    double tagSize = 0.1524; // meters

    // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
    var poseEstConfig =
        new AprilTagPoseEstimator.Config( tagSize, cameraFx, cameraFy, cameraCx, cameraCy);

    var estimator = new AprilTagPoseEstimator(poseEstConfig);

    // Get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture();
    // Set the resolution and frames per second
    camera.setResolution(cameraW, cameraH);
    camera.setFPS(15);

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("Detected", cameraW, cameraH);

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

      for (AprilTagDetection detection : detections) {

        Pose3d tagInFieldFrame; // pose from WPILib resource

        if(aprilTagFieldLayout.getTagPose(detection.getId()).isPresent() && detection.getDecisionMargin() > 50.) // margin < 20 seems bad; margin > 120 are good
        {
          tagInFieldFrame = aprilTagFieldLayout.getTagPose(detection.getId()).get();
        }
        else
        {
          System.out.println("bad id " + detection.getId() + " " + detection.getDecisionMargin());
          continue;
        }

        // remember we saw this tag
        tags.add((long) detection.getId());

        // draw lines around the tag
        for (var i = 0; i <= 3; i++) {
          var j = (i + 1) % 4;
          var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
          var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
          Imgproc.line(mat, pt1, pt2, outlineColor, 2);
          // corners appear as 3 2
          //                   0 1
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

        // determine pose
        Transform3d tagInCameraFrame = estimator.estimate(detection);

        { // draw a frustum in front of the AprilTag
          // use the estimated pose from above before any other transforms  

            // camera same as above but different format for OpenCV
            float[] cameraParm = {(float)cameraFx,   0.f,             (float)cameraCx,
                                    0.f,             (float)cameraFy, (float)cameraCy,
                                    0.f,             0.f,             1.f};
            Mat K = new Mat(3, 3, CvType.CV_32F); // camera matrix
            K.put(0, 0, cameraParm);
  
            MatOfDouble distCoeffs = new MatOfDouble(); // not using any camera distortions so it's empty

            // 3D points of ideal, original corners, flat on the tag, scaled to the actual tag size.
            // Order doesn't matter except must be in same order as the top points so pillars connect right.
            // We could reuse the corners from detector if we know the order (and we do) and avoid redundant
            // variable and recalculation but we'll re-specify them for fun and match the detectors corners order.
            MatOfPoint3f bottom = new MatOfPoint3f(
              new Point3(-1.*tagSize/2.,1.*tagSize/2., 0.),
                  new Point3(1.*tagSize/2., 1.*tagSize/2., 0.),
                  new Point3(1.*tagSize/2., -1.*tagSize/2., 0.),
                  new Point3(-1.*tagSize/2., -1.*tagSize/2., 0.));
  
            // 3D points of the ideal, original corners, in front of the tag to make a frustum, scaled to the actual tag size
            // note that the orientation and size of the face of the box can be controlled by the sign of the "Z"
            // value of the "top" variable.
            // "-" (negative) gives larger top facing straight away from the plane of the tag
            // "+" (positive) gives smaller top facing toward the camera
            MatOfPoint3f top = new MatOfPoint3f( // order doesn't matter except must be in same order as the bottom points
              new Point3(-1.*tagSize/2.,1.*tagSize/2., -0.7*tagSize),
                  new Point3(1.*tagSize/2., 1.*tagSize/2., -0.7*tagSize),
                  new Point3(1.*tagSize/2., -1.*tagSize/2., -0.7*tagSize),
                  new Point3(-1.*tagSize/2., -1.*tagSize/2., -0.7*tagSize));
  
            double[] rotationVector = tagInCameraFrame.getRotation().getQuaternion().toRotationVector().getData(); // 3x1 3 rows 1 col
  
            Mat T = new Mat(3, 1, CvType.CV_64FC1);
            Mat R = new Mat(3, 1, CvType.CV_64FC1);
            T.put(0, 0, tagInCameraFrame.getX(), tagInCameraFrame.getY(), tagInCameraFrame.getZ());
            R.put(0, 0, rotationVector[0], rotationVector[1], rotationVector[2]);
  
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
                    outlineColor,
                    2);
            }
  
            MatOfPoint topCornersTemp = new MatOfPoint();
            topCornersTemp.fromList(topCornerPoints);
  
            ArrayList<MatOfPoint> topCorners = new ArrayList<MatOfPoint>();
            topCorners.add(topCornersTemp);
            
            Imgproc.polylines(mat, topCorners, true, outlineColor, 2);
          } /* end draw a frustum in front of the AprilTag */

        // These transformations are required for the correct robot pose.
        // They might arise from the tag facing the camera thus Pi radians rotated or CCW/CW flipped from the
        // mathematically described pose from the estimator that's what our eyes see. The true rotation
        // has to be used to get the right robot pose. It seems that the T and R from the estimator should
        // take care of all this (it does when drawing the tag and orientation box) since it's consistent
        //  with the OpenCV projectPoints.

            tagInCameraFrame = new Transform3d(
            new Translation3d(
                          tagInCameraFrame.getX(),
                          tagInCameraFrame.getY(),
                          tagInCameraFrame.getZ()),
            new Rotation3d(
                        -tagInCameraFrame.getRotation().getX() - Math.PI,
                        -tagInCameraFrame.getRotation().getY(),
                         tagInCameraFrame.getRotation().getZ() - Math.PI));


        // OpenCV and WPILib estimator layout of axes is EDN and field WPILib is NWU; need x -> -y , y -> -z , z -> x and same for differential rotations
        // pose = CoordinateSystem.convert(pose, CoordinateSystem.EDN(), CoordinateSystem.NWU());
        // WPILib convert is wrong for transforms as of 2023.4.3 so use this patch for now
        {
        // corrected convert
        var from = CoordinateSystem.EDN();
        var to = CoordinateSystem.NWU();
        tagInCameraFrame = new Transform3d(
                  CoordinateSystem.convert(tagInCameraFrame.getTranslation(), from, to),
                  CoordinateSystem.convert(new Rotation3d(), to, from)
                      .plus(CoordinateSystem.convert(tagInCameraFrame.getRotation(), from, to)));
        // end of corrected convert
        }
        
        var // transform to camera from robot chassis center at floor level
        cameraInRobotFrame = new Transform3d(       
        //                      new Translation3d(0., 0., 0.),// camera at center bottom of robot zeros for test data 
        //                      new Rotation3d(0.0, Units.degreesToRadians(0.), Units.degreesToRadians(0.0))); // camera in line with robot chassis
                        new Translation3d(0.2, 0., 0.8),// camera in front of center of robot and above ground
                          new Rotation3d(0., Units.degreesToRadians(-25.), 0.)); // camera in line with robot chassis, pointing up slightly
        // x + roll is camera rolling CCW relative to the robot looking facing the robot
        // y + pitch is camera pointing down relative to the robot. -25 camera points up; +25 points down; sign is correct but backwards of LL
        // z + yaw is camera pointing to the left of robot looking down on it (CCW relative to the robot)

        var // robot in field is the composite of 3 pieces
        robotInFieldFrame = ComputerVisionUtil.objectToRobotPose(tagInFieldFrame,  tagInCameraFrame,  cameraInRobotFrame);

        // end transforms to get the robot pose from this vision tag pose

        // put pose into dashboard
        Rotation3d rot = robotInFieldFrame.getRotation();
        tagsTable
            .getEntry("pose_" + detection.getId())
            .setDoubleArray(
                new double[] {
                  robotInFieldFrame.getX(), robotInFieldFrame.getY(), robotInFieldFrame.getZ(), rot.getX(), rot.getY(), rot.getZ()
                });

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
