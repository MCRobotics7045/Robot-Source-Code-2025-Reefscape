package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import static frc.robot.Constants.Constants.Vision.*;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.littletonrobotics.junction.Logger;


public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public PhotonCamera postionCamera; 


  int FoundID; 
  int SelectedID;
  int Cycle = 0;
  AprilTagFieldLayout fieldLayout;
  PhotonPoseEstimator photonPoseEstimator;
  public Transform3d camPose;
  public static SendableChooser<Integer> AprilTagSelector;
  private int lastCheckedTagId = -1; 
  private boolean warningDisplayed = false; 
  private Pose3d Pose;
  boolean done;
  double yaw;
 
 

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

    postionCamera = new PhotonCamera("PLACEHOLDER");
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
//------------------------------------------------------------------------------------


		fieldLayout =  AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
	
    camPose = new Transform3d(
      new Translation3d(0,0,Units.inchesToMeters(5)), //dont know correct
      new Rotation3d(0,Units.degreesToRadians(15), 0));
      
    photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camPose);

  }

  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // GetResults(2);

  //   if (Cycle >= 50) {
  //     Cycle = 0;
  //     SmartDashboard.putBoolean("Found Tag?", CheckTagID(SelectedID));
  //     PhotonTrackedTarget bestTarget = selectBestTarget();
  //     if (bestTarget != null) {
  //       BestFoundTag = bestTarget.getFiducialId();
  //       yaw = bestTarget.getYaw();
  //     }
  //   }
  //   Cycle = Cycle + 1;
    
    SelectedID = AprilTagSelector.getSelected();
    // SmartDashboard.putNumber("Best Tag Seen", BestFoundTag);
    // SmartDashboard.putNumber("Yaw Of AprilTag", yaw);

  }



  public Pose2d getAprilTagPose2d(int TagId) {
    Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(TagId);
    if (tagPoseOpt.isPresent()) {
      return tagPoseOpt.get().toPose2d();
  } else {
    System.out.print("April Tag Id invalid. Wow that was not a april tag number!!!!!!!!! ");
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

