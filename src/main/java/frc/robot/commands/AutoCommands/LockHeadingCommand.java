package frc.robot.commands.AutoCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Timer;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class LockHeadingCommand extends Command {
    private final SwerveSubsystem swerve;
    private final PhotonCamera camera;
    private final AprilTagFieldLayout fieldLayout;
    // PID for rotation
    private final PIDController rotationPid = new PIDController(2, 0.0, 0.0);
    private static final double ROTATION_TOL_DEG = 0.5;  // how close in degrees to consider "done"
    private static final double ROTATION_CLAMP = 2;    // max rotation speed
    private static final double TIMEOUT_SEC = 3.0;       // fail-safe

    private final Timer timer = new Timer();

    public LockHeadingCommand(SwerveSubsystem swerve, PhotonCamera camera, AprilTagFieldLayout fieldLayout) {
        this.swerve = swerve;
        this.camera = camera;
        this.fieldLayout = fieldLayout;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        rotationPid.reset();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            // if we lost the tag, end
            end(true);
            return;
        }

        // Best target info

        double Rotation = fieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().toPose2d().getRotation().getDegrees();
        double RotationSwap = Rotation + 180;
        var bestTarget = result.getBestTarget();
        double cameraYawDeg = bestTarget.getYaw(); // how many degrees camera sees the tag off-center

        // The robot’s absolute heading from swerve/pigeon
        double robotHeadingDeg = swerve.getState().Pose.getRotation().getDegrees();

        // If camera sees tag is +15 deg, that means the field heading is (robotHeadingDeg + 15)
        double desiredHeadingDeg = robotHeadingDeg - RotationSwap;

        // Run the PID
        double rotationCmd = rotationPid.calculate(robotHeadingDeg, RotationSwap);
        rotationCmd = MathUtil.clamp(rotationCmd, -ROTATION_CLAMP, ROTATION_CLAMP);

        // Just rotate in place (robot-centric)
        swerve.drive(0.0, 0.0, rotationCmd, false);
    }

    @Override
    public boolean isFinished() {
        // If we timed out or if error is small
        boolean timedOut = timer.hasElapsed(TIMEOUT_SEC);
        double errorDeg = rotationPid.getPositionError();
        boolean aligned = Math.abs(errorDeg) < ROTATION_TOL_DEG;
        return timedOut || aligned;
    }

    @Override
    public void end(boolean interrupted) {
        // stop
        swerve.drive(0.0, 0.0, 0.0, false);
    }
}
