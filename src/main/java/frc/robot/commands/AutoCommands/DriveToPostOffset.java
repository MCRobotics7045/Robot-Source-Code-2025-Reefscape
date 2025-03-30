package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * A command that uses 2-axis movement (forward + strafe) to drive to a desired offset from
 * an AprilTag or other target, all in one go.  Ignores rotation (rotation=0).
 */
public class DriveToPostOffset extends Command {
    private final SwerveSubsystem swerve;
    private final PhotonCamera camera;
    private final LEDSubsystem ledSubsystem;
    /**
     * If your camera coordinate system is:
     *   - Z forward
     *   - Y left
     * then these offsets represent how far from the target you want to be along Z,
     * and how far from the target you want to be along Y.
     * 
     * e.g. forwardOffset=1.0 means end up 1 meter from the tag,
     *      sideOffset=0.2 means you want to be 0.2m to the left of the tag
     *        (or if your sign is reversed, that might be right).
     */
    private final double forwardOffset;  
    private final double sideOffset;
    private final PIDController forwardPID = new PIDController(1.0, 0.0, 0);
    private final PIDController sidePID    = new PIDController(1.0, 0.0, 0);
    private static final double FORWARD_TOL = 0.05; // meters
    private static final double SIDE_TOL    = 0.05; // meters
    private static final double FORWARD_CLAMP = 1; // m/s
    private static final double SIDE_CLAMP    = 1; // m/s
    private static final double TIMEOUT_SEC = 4.0;
    private final Timer timer = new Timer();

    List<Integer> blueReefList = java.util.List.of(17,18,19,20,21,22);
    List<Integer> redReefList = java.util.List.of(6,7,8,9,10,11);

    public boolean RedAlliance;
    public DriveToPostOffset(SwerveSubsystem swerve,PhotonCamera camera,double forwardOffset, double sideOffset,LEDSubsystem ledSubsystem) {
        this.swerve = swerve;
        this.camera = camera;
        this.forwardOffset = forwardOffset;
        this.sideOffset = sideOffset;
        this.ledSubsystem = ledSubsystem;
        addRequirements(swerve);



    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        forwardPID.reset();
        sidePID.reset();
        RedAlliance = RobotContainer.IsRed();
        System.out.println("COMMAND RUNNING");
    }

    @Override
    public void execute() {

        //Filter Alliance First 
   
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) {
            end(true);
            System.out.println("COMMAND NO TARGET");
            return;
        }
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        ledSubsystem.breathProgres();
        if (RedAlliance) {
            if(redReefList.contains(bestTarget.getFiducialId())){
                System.out.println("RED REEF TARGET FOUND ID:"+ bestTarget.getFiducialId());
                double rawForwardDist = bestTarget.getBestCameraToTarget().getX();
                double rawSideDist = bestTarget.getBestCameraToTarget().getY();
                double forwardError = -rawForwardDist - forwardOffset;
                double forwardCmd = forwardPID.calculate(forwardError, 0.0);
                forwardCmd = MathUtil.clamp(forwardCmd, -FORWARD_CLAMP, FORWARD_CLAMP);
  
                double sideError = -rawSideDist - sideOffset;
                double sideCmd = sidePID.calculate(sideError, 0.0);
                sideCmd  = MathUtil.clamp(sideCmd, -SIDE_CLAMP, SIDE_CLAMP);
                Logger.recordOutput("FWD PID", forwardCmd);
                Logger.recordOutput("SIDE PID", sideCmd);
                swerve.drive(forwardCmd, sideCmd, 0.0, false);
            } else {
                System.out.println("RED REEF NO TARGET");
              end(true);
              return;
            }
        }else {
            if(blueReefList.contains(bestTarget.getFiducialId())) {
                System.out.println("BLUE REEF TARGET FOUND ID:"+ bestTarget.getFiducialId());
              double rawForwardDist = bestTarget.getBestCameraToTarget().getX();
              double rawSideDist = bestTarget.getBestCameraToTarget().getY();
              double forwardError = -rawForwardDist - forwardOffset;
              double forwardCmd = forwardPID.calculate(forwardError, 0.0);
              forwardCmd = MathUtil.clamp(forwardCmd, -FORWARD_CLAMP, FORWARD_CLAMP);

              double sideError = -rawSideDist - sideOffset;
              double sideCmd = sidePID.calculate(sideError, 0.0);
              sideCmd  = MathUtil.clamp(sideCmd, -SIDE_CLAMP, SIDE_CLAMP);
              Logger.recordOutput("FWD PID", forwardCmd);
              Logger.recordOutput("SIDE PID", sideCmd);
              swerve.drive(forwardCmd, sideCmd, 0.0, false);
            } else {
                System.out.println("BLUE REEF NO TARGET");
              end(true);
              return;
            }
        }

        

        
  
        
    }

    @Override
    public boolean isFinished() {
        boolean timedOut = timer.hasElapsed(TIMEOUT_SEC);
        boolean forwardOk = Math.abs(forwardPID.getPositionError()) < FORWARD_TOL;
        boolean sideOk    = Math.abs(sidePID.getPositionError())    < SIDE_TOL;

        return timedOut || (forwardOk && sideOk);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        if(interrupted) {
            
            ledSubsystem.BlinkBad();
        } else {
            ledSubsystem.BlinkGood();
        }
        swerve.drive(0.0, 0.0, 0.0, true);
    }
}
