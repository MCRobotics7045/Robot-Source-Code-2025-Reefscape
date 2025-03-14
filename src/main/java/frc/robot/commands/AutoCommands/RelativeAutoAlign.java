package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class RelativeAutoAlign extends Command{
  
  private final SwerveSubsystem swerve;
  private final PhotonCamera camera;
  
  
  
  // PIDs: one for forward/back distance, one for rotation
  private final PIDController distancePID = new PIDController(0.5, 0.0, 0.0);
  private final PIDController rotationPID = new PIDController(0.6, 0.0, 0.0);
  private final PIDController strafePID = new PIDController(1, 0, 0);
  // Tolerances for finishing
  private static final double DIST_TOL = 1;  // meters
  private static final double YAW_TOL = 2.0;   // degrees
  
  // For safety/time-limit
  private final Timer timer = new Timer();
  private static final double TIMEOUT = 4.0; // seconds

  public RelativeAutoAlign(SwerveSubsystem swerve, PhotonCamera camera) {
    this.swerve = swerve;
    this.camera = camera;
  

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    // Optionally reset PID integrators:
    distancePID.reset();
    rotationPID.reset();
  }

  @Override
  public void execute() {
    // Grab the latest camera result
    PhotonPipelineResult result = camera.getLatestResult();

    if (!result.hasTargets()) {
      end(true);
      return;
    }

    PhotonTrackedTarget bestTarget = result.getBestTarget();

   
    double yawDegrees = bestTarget.getYaw();

   
    double sideDist = bestTarget.getBestCameraToTarget().getY();


    double rotCmd = rotationPID.calculate(-yawDegrees, 0.0);

    
    double sideDistCMD = strafePID.calculate(-sideDist,0.0);

    swerve.lookAtYaw(0.0, 0.0, 10.0);
    
    // 4) Check if we’re “close enough” or if we timed out
    boolean yawAligned = Math.abs(yawDegrees) < YAW_TOL;
    double distError = (sideDist - 0);
    boolean distAligned = Math.abs(distError) < DIST_TOL;

    if ((yawAligned) || timer.hasElapsed(TIMEOUT)) {
      end(false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    
  }

  @Override
  public boolean isFinished() {
    // We manually end() inside execute()
    return false;
  }
}
