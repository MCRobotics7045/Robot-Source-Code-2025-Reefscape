package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import static edu.wpi.first.units.Units.Meters;
import org.opencv.features2d.Features2d;
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
import static frc.robot.RobotContainer.SENSORS;
import static frc.robot.RobotContainer.SWERVE;
public class VisionSubsystem extends SubsystemBase {

  AprilTagFieldLayout fieldLayout;
  /** Creates a new VisionSubsystem. */
  public static PhotonCamera FRpostionCamera; 
  public Transform3d FRcamPose;
  
  public Field2d fRposField2d = new Field2d();
  public static PhotonCamera FLpostionCamera;
  public Transform3d FLcamPose;
  
  public Field2d fLposField2d = new Field2d();
  public static PhotonCamera BRpostionCamera; 
  public Transform3d BRcamPose;
  
  public Field2d bRposField2d = new Field2d();
  public static PhotonCamera BLpostionCamera; 
  public Transform3d BLcamPose;
   
  public Field2d bLposField2d = new Field2d();


  int FoundID; 
  int SelectedID;
  int Cycle = 0;


  
  public static SendableChooser<Integer> AprilTagSelector;
  private int lastCheckedTagId = -1; 
  private boolean warningDisplayed = false; 
  // private Pose3d Pose;
  boolean done;
  double yaw;
  public boolean upadate = false;
  VisionSystemSim visionSim;
  SimCameraProperties cameraProp;
  private static Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();
  
  
    List<Integer> RedreefTagIDs = List.of(6, 7, 8, 9, 10, 11);
    List<Integer> BluereefTagIDs = List.of(17, 18, 19, 20, 21,22);
    List<Integer> BlueFeedStationIDs = List.of(12,13);
    List<Integer> RedFeedStationIDs = List.of(1,2);
    int BlueAlgeeProcs = 16;
    int RedAlgeeProcs = 3;

    private final AddVisionMeasurement poseConsumer;

    private final PhotonPoseEstimator FRphotonPoseEstimator;
    
      // private final PhotonPoseEstimator  FLphotonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, FLcamPose);
      // FLphotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      // private final PhotonPoseEstimator  BRphotonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, BRcamPose);
      // BRphotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      // private final PhotonPoseEstimator  BLphotonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, BLcamPose);
      // BLphotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    public VisionSubsystem( AddVisionMeasurement poseConsumer) {
      super();
      this.poseConsumer = poseConsumer;

  
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
  
      FRpostionCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  //---------------------------------------------------------------------------------
      
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
              PhotonCameraSim cameraSim = new PhotonCameraSim(FRpostionCamera, cameraProp);
              // X is forward and back and Y is Left and right and Z is Up and Down This is at floor level cause Z=0
              Translation3d robotToCameraTrl = new Translation3d(
                Units.inchesToMeters(0), // convert inches to meters
                Units.inchesToMeters(0),
                Units.inchesToMeters(9.5));
              // 15 Degrees up
              Rotation3d robotToCameraRot = new Rotation3d(
                0,
                Units.degreesToRadians(0), // pitch about Y (radians)
                Units.degreesToRadians(0)  
              );
              Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);
              visionSim.addCamera(cameraSim, robotToCamera);
            
              cameraSim.enableRawStream(true);
              cameraSim.enableProcessedStream(true);
              cameraSim.enableDrawWireframe(true);
              visionSim.getDebugField();
          }
  
  //--------------------------EST POSE---------------------------------------------------
  
  
      fieldLayout =  AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    
      FRcamPose = new Transform3d(
      new Translation3d(
          Units.inchesToMeters(0), // convert inches to meters  
          Units.inchesToMeters(-11.875),
          Units.inchesToMeters(9.5)),
      new Rotation3d(
          0,
          0,
          0
  
      )
  );
       FLcamPose = new Transform3d(
      new Translation3d(
          Units.inchesToMeters(9.875), // convert inches to meters  
          Units.inchesToMeters(11.875),
          Units.inchesToMeters(9.5)),
      new Rotation3d(
          0,
          Units.degreesToRadians(-90),
          Units.degreesToRadians(-25)
  
      )
  );

 

  
  
        
      

    FRphotonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, FRcamPose);

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
      var Results = FRpostionCamera.getLatestResult();
  
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
      SmartDashboard.putData("FR" ,fRposField2d);
      SmartDashboard.putData("FL" ,fLposField2d);
      SmartDashboard.putData("BR" , bRposField2d);
      SmartDashboard.putData("BL",bLposField2d);
      SmartDashboard.putNumber("bestTargetID", bestTargetID());
      if (Robot.isSimulation()) {
        visionSim.update(SWERVE.getState().Pose);
      }
    }
  
   
  public void useCamera() {
    // integrateCamera(UseFL,FLpostionCamera,FLphotonPoseEstimator,fLposField2d,maxDistance);
    integrateCamera(UseFR,FRpostionCamera,FRphotonPoseEstimator,fRposField2d,maxDistance, fieldLayout);
    // integrateCamera(UseBL,BLpostionCamera,BLphotonPoseEstimator,bLposField2d,maxDistance);
    // integrateCamera(UseBR,BRpostionCamera,BRphotonPoseEstimator,bRposField2d,maxDistance);
  }
    

private static Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose,
      List<PhotonTrackedTarget> targets,
      AprilTagFieldLayout fieldLayout) {

    var estStdDevs = SINGLE_TAG_STD_DEVS;
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = fieldLayout.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty())
        continue;
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    if (numTags == 0) {
      return estStdDevs;
    }
    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1) {
      estStdDevs = MULTI_TAG_STD_DEVS;
    }

    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > SINGLE_TAG_DISTANCE_THRESHOLD.in(Meters)) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    }
    return estStdDevs;
  }


  public void integrateCamera(
    boolean useCamera, 
    PhotonCamera camera, 
    PhotonPoseEstimator estimator,
    Field2d photonField, 
    double maxDistance,
    AprilTagFieldLayout fieldLayout
) {
   
    List<PhotonPipelineResult> cameraPipeline = camera.getAllUnreadResults();
    for (int i = 0; i < cameraPipeline.size(); i++) {

        Optional<EstimatedRobotPose> photonPose = estimator.update(cameraPipeline.get(i));
        if (photonPose.isPresent()) {
            photonField.setRobotPose(photonPose.get().estimatedPose.toPose2d());
            double tag0Dist = cameraPipeline.get(i).getBestTarget()
                .bestCameraToTarget.getTranslation().getNorm();
            double poseAmbiguity = cameraPipeline.get(i).getBestTarget().getPoseAmbiguity();
            var stdDevs = getEstimationStdDevs(
                    photonPose.get().estimatedPose.toPose2d(),
                    cameraPipeline.get(i).getTargets(),
                    fieldLayout);

            Pose2d newPose = new Pose2d(
              new Translation2d(
                photonPose.get().estimatedPose.toPose2d().getX(),
                photonPose.get().estimatedPose.toPose2d().getY()
              ),
              new Rotation2d(
                SENSORS.PigeonIMU.getAngle()
              )
            );
            if (useCamera && tag0Dist < maxDistance && poseAmbiguity < 0.05) { 
                poseConsumer.addVisionMeasurement(
                        photonPose.get().estimatedPose.toPose2d(),
                        photonPose.get().timestampSeconds,
                        stdDevs
                    );

               
                }
            }
        }
    }


  public boolean seesTagID(int TargetID) {
    var result = FRpostionCamera.getLatestResult();
    if (!result.hasTargets()) {
      return false;
    } else {
      if (result.getBestTarget().getFiducialId() == TargetID) {
        return true;
      }
      return false;
    }
  }
  public double getBestTagYaw(int TargetID) {
    var result = FRpostionCamera.getLatestResult();
    if (!result.hasTargets()) {
      return 0;
    } else {
      if (result.getBestTarget().getFiducialId() == TargetID) {
        return result.getBestTarget().getYaw();
      }
      return 0;
    }
  }

  public int bestTargetID() {
    var result = FRpostionCamera.getLatestResult();
    if (!result.hasTargets()) {
      return 0;
    } else {
      return result.getBestTarget().getFiducialId();
    }
    
  }
  public Pose2d getBestReefAprilTagPose() {
    List<Integer> reefTagIDs;
    
    if (RobotContainer.IsRed()) { // We are on Red Allinace
      reefTagIDs = RedreefTagIDs;
    } else {
      reefTagIDs = BluereefTagIDs;
    }

    var result = FRpostionCamera.getLatestResult();

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

  @FunctionalInterface
  public interface AddVisionMeasurement {
    void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

}  
    


