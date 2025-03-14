package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Timer;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class DriveForwardToTag extends Command {
    private final SwerveSubsystem swerve;
    private final PhotonCamera camera;

    // Distance from camera lens to tag we want to end at
    private final double desiredDistanceMeters;

    private final PIDController forwardPid = new PIDController(2, 0.0, 0.0);
    private static final double FORWARD_TOL = 0.5;  // how close in meters
    private static final double FORWARD_CLAMP = .6;
    private static final double TIMEOUT_SEC = 4.0;

    private final Timer timer = new Timer();

    public DriveForwardToTag(SwerveSubsystem swerve, PhotonCamera camera, double desiredDistanceMeters) {
        this.swerve = swerve;
        this.camera = camera;
        this.desiredDistanceMeters = desiredDistanceMeters;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        forwardPid.reset();
        timer.reset();
        timer.start();
        System.out.print("DriveForwardToTag Called");
    }

    @Override
    public void execute() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            end(true);
            return;
        }
        var bestTarget = result.getBestTarget();
        double currentDist = result.getBestTarget().getBestCameraToTarget().getX();

        double forwardCmd = forwardPid.calculate(-currentDist, 0);
        forwardCmd = MathUtil.clamp(forwardCmd, -FORWARD_CLAMP, FORWARD_CLAMP);

        
        System.out.print("DriveForwardToTag Calculated at : " + currentDist);


        swerve.drive(forwardCmd, 0.0, 0.0, false);
    }

    @Override
    public boolean isFinished() {
        boolean timedOut = timer.hasElapsed(TIMEOUT_SEC);
        double error = Math.abs(forwardPid.getPositionError());
        boolean inRange = error < FORWARD_TOL;
        return timedOut || inRange;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0.0, 0.0, 0.0, false);
    }
}
