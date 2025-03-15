package frc.robot.commands.AutoCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class LockHeadingCommand extends Command {
    private final SwerveSubsystem swerve;
    private final PhotonCamera camera;
    private final VisionSubsystem vision;
    public static AprilTagFieldLayout kTagLayouts = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    
        


    // PID for rotation
    private final PIDController rotationPid = new PIDController(3, 0.0, 0.0);
    private static final double ROTATION_TOL_DEG = 1;  // how close in degrees to consider "done"
    private static final double ROTATION_CLAMP = 2;    // max rotation speed
    private static final double TIMEOUT_SEC = 3.0;       // fail-safe

    private final Timer timer = new Timer();
    List<Integer> ReefTopsideints = java.util.List.of(8,9,19,20);
    List<Integer> ReefUnderSideInts = java.util.List.of(6,11,17,22);
    List<Integer> ReefLeftSide = java.util.List.of(10,18);
    List<Integer> ReefRightSide = java.util.List.of(7,21);


    public LockHeadingCommand(SwerveSubsystem swerve, PhotonCamera camera,VisionSubsystem vision) {
        this.swerve = swerve;
        this.camera = camera;
        this.vision = vision;
        addRequirements(swerve);
        
    }

    @Override
    public void initialize() {
        
        rotationPid.reset();
        timer.reset();
        timer.start();
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            end(true);
            return;
        }
        vision.AlignCommandSelectedTag  = result.getBestTarget().getFiducialId();
    }

    @Override
    public void execute() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
               
            end(true);
            return;
        }
        System.out.println(vision.AlignCommandSelectedTag);


        double Rotation = kTagLayouts.getTagPose(vision.AlignCommandSelectedTag).get().toPose2d().getRotation().getDegrees();
        double RotationSwap = FlipDegrees(Rotation);
        double robotHeadingDeg = swerve.getState().Pose.getRotation().getDegrees();
        


        double rotationCmd = rotationPid.calculate(robotHeadingDeg, RotationSwap);
        rotationCmd = MathUtil.clamp(rotationCmd, -ROTATION_CLAMP, ROTATION_CLAMP);
        SmartDashboard.putNumber("ClosedLoop Roation", rotationPid.getPositionError());
        SmartDashboard.putNumber("ClosedLoop SetPoint", rotationPid.getSetpoint());
        SmartDashboard.putNumber("Pigeon Data", swerve.getState().Pose.getRotation().getDegrees());
        swerve.drive(0.0, 0.0, rotationCmd, false);
    }

    @Override
    public boolean isFinished() {
      
        boolean timedOut = timer.hasElapsed(TIMEOUT_SEC);
        double errorDeg = rotationPid.getPositionError();
        boolean aligned = Math.abs(errorDeg) < ROTATION_TOL_DEG;
        return aligned ;
    }

    @Override
    public void end(boolean interrupted) {

        swerve.drive(0.0, 0.0, 0.0, false);
    }


    public double FlipDegrees(Double rotation) {
      List<Integer> reefTagIDs;
      boolean isRed = RobotContainer.IsRed();
      double rotaionFlip;


     if (ReefTopsideints.contains(vision.AlignCommandSelectedTag)) {
      return rotaionFlip = rotation-180;
     }else if (ReefUnderSideInts.contains(vision.AlignCommandSelectedTag)) {
      return rotaionFlip = rotation+180;
     } else if (ReefLeftSide.contains(vision.AlignCommandSelectedTag)) {
        if (swerve.getState().Pose.getRotation().getDegrees() < 0) {
            return -0.0001;
          } else {
            return 0;
          }
     } else if (ReefRightSide.contains(vision.AlignCommandSelectedTag)) {
      if (swerve.getState().Pose.getRotation().getDegrees() < 0) {
        return -179.999999;
      } else {
        return 180;
      }
     }

     return 0;
    }
}
