package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class AlignToPost extends Command {
    private final SwerveSubsystem swerve;

    private final PIDController strafePid = new PIDController(3, 0.0, 0.0);
    private static final double STRAFE_TOL = 0.05;   // how close in meters to consider "centered"
    private static final double STRAFE_CLAMP = 0.2;  // max strafe speed
    private static final double TIMEOUT_SEC = 3.0;
    private static double offsetedPose = 0.3;
    private final Timer timer = new Timer();
    private boolean isLeftPost;
    public AlignToPost(SwerveSubsystem swerve, boolean isLeftPost) {
        this.swerve = swerve;
        this.isLeftPost = isLeftPost;
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
  
        if (isLeftPost) {offsetedPose = -offsetedPose;} 


        double strafeCmd = strafePid.calculate(offsetedPose, 0.0);
        strafeCmd = MathUtil.clamp(strafeCmd, -STRAFE_CLAMP, STRAFE_CLAMP);

    
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
