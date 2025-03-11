package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import org.photonvision.EstimatedRobotPose;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
// import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Constants.Vision.*;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public static PhotonCamera postionCamera; 


  int FoundID; 
  int SelectedID;
  int Cycle = 0;
  AprilTagFieldLayout fieldLayout;
  private final PhotonPoseEstimator photonPoseEstimator;
  public Transform3d camPose;
  public static SendableChooser<Integer> AprilTagSelector;
  private int lastCheckedTagId = -1; 
  private boolean warningDisplayed = false; 
  // private Pose3d Pose;
  boolean done;
  double yaw;
 
  VisionSystemSim visionSim;
  SimCameraProperties cameraProp;
  private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();


  List<Integer> RedreefTagIDs = List.of(6, 7, 8, 9, 10, 11);
  List<Integer> BluereefTagIDs = List.of(17, 18, 19, 20, 21,22);
  List<Integer> BlueFeedStationIDs = List.of(12,13);
  List<Integer> RedFeedStationIDs = List.of(1,2);
  int BlueAlgeeProcs = 16;
  int RedAlgeeProcs = 3;
  public VisionSubsystem() {
    super();


//------------------------------------------------------------------------------------
   AprilTagSelector = new SendableChooser<Integer>();

   for (int i = 1; i <= 22; i++) {
    if (i == 1) {
        AprilTagSelector.setDefaultOption("Tag " + i, i); 
    } else {
        AprilTagSelector.addOption("Tag " + i, i);
    }
  }
  SmartDashboard.putData("AprilTag Selection", AprilTagSelector);

//----------------------------------------------------------------------------------

    postionCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
//---------------------------------------------------------------------------------
    var result = postionCamera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      int StartupTargetID = target.getFiducialId();
      System.out.print("Target Found:");
      System.out.println(StartupTargetID);
      SmartDashboard.putNumber("April Tag Found", StartupTargetID);
    } else {
      System.out.println("Warning No Tag Found");
    }


//Simulaton 


        if (Robot.isSimulation()) {
          
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(getTagLayout());
            cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            cameraProp.setCalibError(0.25, 0.08);
            cameraProp.setFPS(32);
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);
            System.out.println("Cam Set Up Comp");
            PhotonCameraSim cameraSim = new PhotonCameraSim(postionCamera, cameraProp);
            // X is forward and back and Y is Left and right and Z is Up and Down This is at floor level cause Z=0
            Translation3d robotToCameraTrl = new Translation3d(
              Units.inchesToMeters(9.875), // convert inches to meters
              Units.inchesToMeters(-11.875),
              Units.inchesToMeters(9.5));
            // 15 Degrees up
            Rotation3d robotToCameraRot = new Rotation3d(
              0,
              Units.degreesToRadians(-25), // pitch about Y (radians)
              Units.degreesToRadians(25.0)  
            );
            Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);
            visionSim.addCamera(cameraSim, robotToCamera);
          
            cameraSim.enableRawStream(true);
            cameraSim.enableProcessedStream(true);
            cameraSim.enableDrawWireframe(true);
        }

//--------------------------EST POSE---------------------------------------------------


		fieldLayout =  AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
	
    camPose = new Transform3d(
    new Translation3d(
        Units.inchesToMeters(9.875), // convert inches to meters  
        Units.inchesToMeters(-11.875),
        Units.inchesToMeters(9.5)),
    new Rotation3d(
        0,
        Units.degreesToRadians(-90),
        Units.degreesToRadians(25)

    )

   
);

      
    photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camPose);

  }

  
  public void simulationPeriodic(Pose2d groundTruthRobotPose) {
    visionSim.update(groundTruthRobotPose);
}

  public Pose2d CheckForAlignCommand(boolean isFeedStation) {
    List<Integer> reefTagIDs = new ArrayList<>();

    if (RobotContainer.IsRed()) { //red
      if (isFeedStation) { //Target Red feed
        reefTagIDs = RedFeedStationIDs;
      } else { //red Procsesor 
        reefTagIDs.add(RedAlgeeProcs);
      }
    } else { //blue
      if (isFeedStation) { //Target blue feed
        reefTagIDs = BlueFeedStationIDs;
      } else { //blue Procsesor 
        reefTagIDs.add(BlueAlgeeProcs);
      }
    }
    var Results = postionCamera.getLatestResult();

    if (!Results.hasTargets()) {
      return null;
    }
    Pose2d bestpose = null;
    for (var target : Results.getTargets()) { 
      int targetID = target.getFiducialId();

      if (reefTagIDs.contains(targetID)) {
        bestpose = fieldLayout.getTagPose(targetID).get().toPose2d();
      } else {
        System.out.print("No Good Tag. Check For Align Has tag but incorrect.");
      }

      
    }

    return bestpose;
  }

  @Override
  public void periodic() {

    
    
    PhotonPipelineResult result = postionCamera.getLatestResult();
    if (!result.hasTargets()) {
      latestEstimatedPose = Optional.empty();
      return;
    }
    
    SmartDashboard.putNumber("Tag Seen", result.getBestTarget().getFiducialId());
    Optional<EstimatedRobotPose> poseOPT = photonPoseEstimator.update(result);

    if (poseOPT.isPresent()) {
      latestEstimatedPose = poseOPT;
      EstimatedRobotPose estPose = poseOPT.get();

    } else {
      latestEstimatedPose = Optional.empty();
    }

    
  }

  public Optional<EstimatedRobotPose> getLatestPose() {
    return latestEstimatedPose;
  }


  public static List<Integer> getAllSeenTags() {
    var result = postionCamera.getLatestResult();

    if (!result.hasTargets()) {
        return List.of(); // No tags detected, return empty list
    }

    return result.getTargets().stream()
            .map(PhotonTrackedTarget::getFiducialId)
            .collect(Collectors.toList()); 
}


  public Pose2d getAprilTagPose2d(int TagId) {
    Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(TagId);
    if (tagPoseOpt.isPresent()) {
      return tagPoseOpt.get().toPose2d();
  } else {
    return null;
  }
  }

  // Declare these as class-level variables to persist across function calls

  private PhotonTrackedTarget selectBestTarget() {
    var result = postionCamera.getLatestResult();
    if (!result.hasTargets()) {
        return null; 
    }
    return result.getBestTarget(); 
  }


  public Pose2d getBestReefAprilTagPose() {
    List<Integer> reefTagIDs;
    
    if (RobotContainer.IsRed()) { // We are on Red Allinace
      reefTagIDs = RedreefTagIDs;
    } else {
      reefTagIDs = BluereefTagIDs;
    }

    var result = postionCamera.getLatestResult();

    if (!result.hasTargets()) {
      return null;
    }

    Pose2d bestpose = null;
    for (var target : result.getTargets()) {

      int targetID = target.getFiducialId();
      if(reefTagIDs.contains(targetID)) {
        Pose2d tagPose2d = fieldLayout.getTagPose(targetID).get().toPose2d();
        bestpose = tagPose2d;
        System.out.print(targetID);
      }
      
      
    }
    
    

    return bestpose;
  }

  public double getBestTagYaw() {
    var results = postionCamera.getLatestResult();
    double Yaw = 0;
    if (results.hasTargets()) {
      PhotonTrackedTarget Tag = results.getBestTarget();
      Yaw = Tag.getYaw();
    } 

    if (yaw == 0) {
      System.out.print("Warning getBestTagYaw() was called but results doesnt have Target");
    }
    
    return yaw;
  }
  public double FindPitch() {
    var result = postionCamera.getLatestResult();
    if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        double Pitch = target.getPitch();  
        double correctedPitch = Pitch + 10;
        return Pitch;
    }
    return 0.0;  // Or some other default value if no target is found
}
  
  public boolean CheckTagID(int TagId) {
    var result = postionCamera.getLatestResult();
    int FoundID = -1; 
    int CurrentID;
    
    if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        FoundID = target.getFiducialId(); 
        CurrentID = FoundID;
        if (FoundID == TagId) {
            lastCheckedTagId = TagId; 
            warningDisplayed = false;
            return true; 
        }
    }
    if (FoundID != TagId) {
        if (lastCheckedTagId != TagId) {
            lastCheckedTagId = TagId;
            warningDisplayed = false; 
        }
        if (!warningDisplayed) {
            System.out.print("Warning April Tag: ");
            System.out.print(TagId);
            System.out.println(" Not found");
            warningDisplayed = true; 
        }
        return false; 
    }
    return false; 
  }

  public boolean CheckVisionHealth() {
    var result = postionCamera.getLatestResult();
    int HealthScore = 0;
    if (result.getTimestampSeconds() > 0) {
      HealthScore += 0.1;
    } else if (result.hasTargets()) {
      HealthScore += 0.1;
    }
    if (HealthScore >= 1) {
      //PASS
      return true;
    } else {
      return false;
    }
  }


  // public double FindTagIDyaw(int TagId) {
  //   var result = poseCam.getLatestResult();
  //   double targetYaw = 0.0;

  //   if (result.hasTargets()) {
  //     for (var target : result.getTargets()) {
  //         if (target.getFiducialId() == TagId) {
  //             targetYaw = target.getYaw();
              
  //         }
  //     }
  //   } else {
  //     targetYaw = 0.0;
  //   }
  //   return targetYaw;

  // } 

 
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        var result = postionCamera.getLatestResult();
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(result);
  }
}

