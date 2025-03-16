package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * A command that uses 2-axis movement (forward + strafe) to drive to a desired offset from
 * an AprilTag or other target, all in one go.  Ignores rotation (rotation=0).
 */
public class DriveToPostOffset extends Command {
    private final SwerveSubsystem swerve;
    private final PhotonCamera camera;

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
    private final PIDController forwardPID = new PIDController(0.5, 0.0, 0.0);
    private final PIDController sidePID    = new PIDController(1.0, 0.0, 0.0);
    private static final double FORWARD_TOL = 0.05; // meters
    private static final double SIDE_TOL    = 0.05; // meters
    private static final double FORWARD_CLAMP = 1.0; // m/s
    private static final double SIDE_CLAMP    = 1.0; // m/s
    private static final double TIMEOUT_SEC = 4.0;
    private final Timer timer = new Timer();

    List<Integer> blueReefList = java.util.List.of(17,18,19,20,21,22);
    List<Integer> redReefList = java.util.List.of(6,7,8,9,10,11);

    public boolean RedAlliance;
    public DriveToPostOffset(SwerveSubsystem swerve,PhotonCamera camera,double forwardOffset, double sideOffset) {
        this.swerve = swerve;
        this.camera = camera;
        this.forwardOffset = forwardOffset;
        this.sideOffset = sideOffset;

        addRequirements(swerve);



    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        forwardPID.reset();
        sidePID.reset();
        RedAlliance = RobotContainer.IsRed();
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) {
            end(true);
            return;
        }
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        if (RedAlliance) {
            if(redReefList.contains(bestTarget.getFiducialId())){
              double rawForwardDist = bestTarget.getBestCameraToTarget().getX();
              double rawSideDist = bestTarget.getBestCameraToTarget().getY();
              double forwardError = -rawForwardDist - forwardOffset;
              double forwardCmd = forwardPID.calculate(forwardError, 0.0);
              forwardCmd = MathUtil.clamp(forwardCmd, -FORWARD_CLAMP, FORWARD_CLAMP);

              double sideError = -rawSideDist - sideOffset;
              double sideCmd = sidePID.calculate(sideError, 0.0);
              sideCmd  = MathUtil.clamp(sideCmd, -SIDE_CLAMP, SIDE_CLAMP);

              swerve.drive(forwardCmd, sideCmd, 0.0, false);
            } else {
              end(true);
              return;
            }
        }else {
            if(blueReefList.contains(bestTarget.getFiducialId())) {
              double rawForwardDist = bestTarget.getBestCameraToTarget().getX();
              double rawSideDist = bestTarget.getBestCameraToTarget().getY();
              double forwardError = -rawForwardDist - forwardOffset;
              double forwardCmd = forwardPID.calculate(forwardError, 0.0);
              forwardCmd = MathUtil.clamp(forwardCmd, -FORWARD_CLAMP, FORWARD_CLAMP);

              double sideError = -rawSideDist - sideOffset;
              double sideCmd = sidePID.calculate(sideError, 0.0);
              sideCmd  = MathUtil.clamp(sideCmd, -SIDE_CLAMP, SIDE_CLAMP);
              
              swerve.drive(forwardCmd, sideCmd, 0.0, false);
            } else {
              end(true);
              return;
            }
        }

        

        
  
        
    }

    @Override
    public boolean isFinished() {
        boolean timedOut = timer.hasElapsed(TIMEOUT_SEC);

        // Check if forward and side errors are small enough
        // If you used atSetpoint(), you could do forwardPID.atSetpoint() etc.
        boolean forwardOk = Math.abs(forwardPID.getPositionError()) < FORWARD_TOL;
        boolean sideOk    = Math.abs(sidePID.getPositionError())    < SIDE_TOL;

        return timedOut || (forwardOk && sideOk);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        swerve.drive(0.0, 0.0, 0.0, false);
    }
}
