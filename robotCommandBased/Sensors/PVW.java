package frc.robot.Sensors;

import java.io.IOException;
import java.util.List;
import java.util.function.Consumer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Sensors.PhotonPoseEstimator.PoseStrategy;

public class PVW {
    
    
  PhotonCamera camera;
  boolean ChargedUpTagLayout = true; // false is use custom deploy of layout
  NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");

    AprilTagFieldLayout aprilTagFieldLayout;
    // transform from camera to robot chassis center which is located on the ground
    Transform3d cameraToRobot = new Transform3d(
                         new Translation3d(-0.2, 0.0, -0.8), // camera in front of center of robot and above ground          
                        //  new Translation3d(0., 0., 0.), // camera at center of robot - good for testing other transforms          
                         new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0))); // camera in line with robot chassis
    PhotonPoseEstimator photonPoseEstimator;
    
    public PVW()
    {
// Tag positions
// tag rotation is CCW looking down on field from the ceiling.
// rotating around Z, 0 degrees is parallel to Y and facing down field or +X. 30 degrees is still
// facing down field +X and a little facing into the +Y across the field

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

    // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            camera,
            cameraToRobot.inverse());
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

    // Change this to match the name of your camera
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
   }

   public void PVacquire()
   {
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();

    // Check if the latest result has any targets.
    boolean hasTargets = result.hasTargets();

    if(!hasTargets) return;

    // System.out.println("PV targets " + result.getTargets().toString());
    /*
PV targets 
[PhotonTrackedTarget{yaw=6.152562590623817, pitch=10.332832851106183, area=5.461263020833333, skew=0.0, fiducialId=1,
     cameraToTarget=Transform3d(Translation3d(X: 0.74, Y: -0.06, Z: 0.09), Rotation3d(Quaternion(-0.2726261258379843, -0.07106175523592184, -0.03804968920027729, -0.9587374216150759))),
     targetCorners=[(310.80324451354096,65.83606084080915), (440.9999761105731,53.99999757968206), (454.1966944513028,199.16393915919085), (323.9999628542707,211.00000242031794)]},
PhotonTrackedTarget{yaw=-7.508814114904784, pitch=6.939642661760454, area=3.12451171875, skew=0.0, fiducialId=8,
     cameraToTarget=Transform3d(Translation3d(X: 0.87, Y: 0.11, Z: 0.06), Rotation3d(Quaternion(-0.317297278974922, -0.07412982180735696, -0.05097077054253319, -0.9440493561378137))),
     targetCorners=[(199.52510255473152,102.60311480911115), (285.99997420259956,99.00000041642924), (291.47483641011223,230.39688519088884), (204.9999647622442,233.99999958357074)]}]

PV robot pose Pose3d(Translation3d(X: 14.80, Y: 0.73, Z: -0.56), Rotation3d(Quaternion(-0.9587374216150759, -0.03804968920027729, 0.07106175523592184, -0.2726261258379843)))
     */

    // Get a list of currently tracked targets.
    List<PhotonTrackedTarget> targets = result.getTargets();

    // Get the current best target.
    PhotonTrackedTarget target = result.getBestTarget();

    // Get information from target.
    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double area = target.getArea();
    double skew = target.getSkew();
    Transform3d pose = target.getBestCameraToTarget();

    List<TargetCorner> corners = target.getDetectedCorners(); // RPi doc has this wrong

    // Get information from target.
    int targetID = target.getFiducialId();
    double poseAmbiguity = target.getPoseAmbiguity();
    Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

    // // Capture pre-process camera stream image
    // camera.takeInputSnapshot();

    // // Capture post-process camera stream image
    // camera.takeOutputSnapshot();

    // Calculate robot's field relative pose
    Pose3d robotInField = PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(),
                aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
                cameraToRobot);

    // System.out.println("PV robot pose " + robotInField);

    // put out to NetworkTables this tag's robot pose
    tagsTable
    .getEntry("robotpose_" + target.getFiducialId())
    .setDoubleArray(
        new double[] {
            robotInField.getTranslation().getX(), robotInField.getTranslation().getY(), robotInField.getTranslation().getZ(),
            robotInField.getRotation().getQuaternion().getW(), robotInField.getRotation().getQuaternion().getX(),
            robotInField.getRotation().getQuaternion().getY(), robotInField.getRotation().getQuaternion().getZ()
        });
        
    System.out.println("angle " + target.getFiducialId()
                            +       Units.radiansToDegrees(robotInField.getRotation().getX())
                            + " " + Units.radiansToDegrees(robotInField.getRotation().getY())
                            + " " + Units.radiansToDegrees(robotInField.getRotation().getZ())
                            + " " + Units.radiansToDegrees(robotInField.getRotation().getAngle()) );
    }
}
