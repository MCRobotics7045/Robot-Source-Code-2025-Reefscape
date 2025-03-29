package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveOffset extends Command {
    private final SwerveSubsystem swerve;
    private final PhotonCamera camera;
    private final boolean pushLeft;

    private final Timer timer = new Timer();
    private static final double TIMEOUT_SEC = 5.0;
    private static final double PUSH_SPEED = 0.15; // meters/second side speed
    private double finalSideDist = 0.0; // for logging

    /**
     * @param swerve The swerve subsystem
     * @param camera The PhotonVision camera
     * @param pushLeft If true, push left; if false, push right
     */
    public DriveOffset(SwerveSubsystem swerve, PhotonCamera camera, boolean pushLeft) {
        this.swerve = swerve;
        this.camera = camera;
        this.pushLeft = pushLeft;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        finalSideDist = 0.0;
    }

    @Override
    public void execute() {
        double sideCmd = pushLeft ? PUSH_SPEED : -PUSH_SPEED;
        swerve.drive(0.0, sideCmd, 0.0,  false);

       
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            double sideDist = bestTarget.getBestCameraToTarget().getY();
            finalSideDist = sideDist;
        }
    }

    @Override
    public boolean isFinished() {
        // Stop automatically after TIMEOUT_SEC, or the user can interrupt
        return timer.hasElapsed(TIMEOUT_SEC);
    }

    @Override
    public void end(boolean interrupted) {
        
        swerve.drive(0.0, 0.0, 0.0, false);
        System.out.println("PushLeftRightTune ended. Final side offset = " + finalSideDist);
        Logger.recordOutput("Auto Align Tune Value:",finalSideDist);
    }
}
