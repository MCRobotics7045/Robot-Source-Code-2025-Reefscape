package frc.robot.Simulation;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;

import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.RobotContainer.VISION;
import static frc.robot.Constants.Constants.Vision.*;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
public class SimulationTele {

    PhotonCamera SimCam = VISION.poseCam;
    boolean IsSimReady = false; 
    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public VisionSystemSim visionSim;
    SimCameraProperties cameraProp;
    public SimulationTele() {
        if (Robot.isSimulation()) {
            System.out.println("Sim Started");
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(getTagLayout());
            cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            cameraProp.setCalibError(0.25, 0.08);
            cameraProp.setFPS(32);
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);
            System.out.println("Cam Set Up Comp");
            PhotonCameraSim cameraSim = new PhotonCameraSim(SimCam, cameraProp);
            // X is forward and back and Y is Left and right and Z is Up and Down This is at floor level cause Z=0
            Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
            // 15 Degrees up
            Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
            Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);
            visionSim.addCamera(cameraSim, robotToCamera);
            System.out.println("Vision Sim ready");
            cameraSim.enableRawStream(true);
            cameraSim.enableProcessedStream(true);
            cameraSim.enableDrawWireframe(true);
            IsSimReady = true; 
        }

        Logger.recordOutput("IsSimReady", IsSimReady);
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
 
    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Robot speeds for general checking */
    private final NetworkTable driveStats = inst.getTable("Drive");
    private final DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
    private final DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
    private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
    private final DoublePublisher odomPeriod = driveStats.getDoubleTopic("Odometry Period").publish();

    /* Keep a reference of the last pose to calculate the speeds */
    private Pose2d m_lastPose = new Pose2d();
    private double lastTime = Utils.getCurrentTimeSeconds();
    
    /* Mechanisms to represent the swerve module states */
    // private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
    //     new Mechanism2d(1, 1),
    //     new Mechanism2d(1, 1),
    //     new Mechanism2d(1, 1),
    //     new Mechanism2d(1, 1),
    // };
    /* A direction and length changing ligament for speed representation */
    // private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
    //     m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    //     m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    //     m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    //     m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    // };
    // /* A direction changing and length constant ligament for module direction */
    // private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
    //     m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
    //         .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    //     m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
    //         .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    //     m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
    //         .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    //     m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
    //         .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    // };

   
    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the pose */
        Pose2d pose = SWERVE.getPose();
        fieldTypePub.set("Field2d");
        if (Robot.isSimulation()) {
            visionSim.update(pose);
            visionSim.getDebugField();
        }
        fieldPub.set(new double[] {
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees()
        });

        /* Telemeterize the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
        Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
        m_lastPose = pose;

        Translation2d velocities = distanceDiff.div(diffTime);

        speed.set(velocities.getNorm());
        velocityX.set(velocities.getX());
        velocityY.set(velocities.getY());
        odomPeriod.set(state.OdometryPeriod);

        /* Telemeterize the module's states */
        // if (Robot.isSimulation()) {
        //     for (int i = 0; i < 4; ++i) {
        //     m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
        //     m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
        //     m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

        //     SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        // }
        // }
        
        
        
    }

    public Pose2d getCurrentPose() {
        return m_lastPose;
    }

    public void resetPose(Pose2d newPose) {
        m_lastPose = newPose;  // Update the internal last pose reference
        lastTime = Utils.getCurrentTimeSeconds();  // Reset the time for velocity calculations

        // Publish the new pose to NetworkTables (for immediate telemetry update)
        fieldPub.set(new double[] {
            newPose.getX(),
            newPose.getY(),
            newPose.getRotation().getDegrees()
        });
    }
    
}
