package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class StrafeToTagCenter extends Command {
    private final SwerveSubsystem swerve;
    private final PhotonCamera camera;

    private final PIDController strafePid = new PIDController(2, 0.0, 0.0);
    private static final double STRAFE_TOL = 0.05;   // how close in meters to consider "centered"
    private static final double STRAFE_CLAMP = 2;  // max strafe speed
    private static final double TIMEOUT_SEC = 3.0;

    private final Timer timer = new Timer();

    public StrafeToTagCenter(SwerveSubsystem swerve, PhotonCamera camera) {
        this.swerve = swerve;
        this.camera = camera;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        strafePid.reset();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            end(true);
            return;
        }
        var bestTarget = result.getBestTarget();

        // The Y offset from the camera (assuming +Y means to the left or right).
        // This depends heavily on how your camera is oriented.
        double lateralMeters = bestTarget.getBestCameraToTarget().getY();

        double strafeCmd = strafePid.calculate(-lateralMeters, 0.0);
        strafeCmd = MathUtil.clamp(strafeCmd, -STRAFE_CLAMP, STRAFE_CLAMP);

        // We keep rotation = 0, forward/back = 0, just strafe
        swerve.drive(0.0, strafeCmd, 0.0, false);
    }

    @Override
    public boolean isFinished() {
        boolean timedOut = timer.hasElapsed(TIMEOUT_SEC);
        double error = Math.abs(strafePid.getPositionError());
        boolean centered = error < STRAFE_TOL;
        return timedOut || centered;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0.0, 0.0, 0.0, false);
    }
}
