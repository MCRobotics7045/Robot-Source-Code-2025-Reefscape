package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import java.util.List;

public class SelectiveDriveToPostOffset extends Command {
    private final SwerveSubsystem swerve;
    private final PhotonCamera camera;
    private final LEDSubsystem ledSubsystem;
    private final double forwardOffset;
    private final double sideOffset;
    private final PIDController forwardPID = new PIDController(1.0, 0.0, 0);
    private final PIDController sidePID = new PIDController(1.0, 0.0, 0);
    private static final double FORWARD_TOL = 0.05;
    private static final double SIDE_TOL = 0.05;
    private static final double FORWARD_CLAMP = 1.0;
    private static final double SIDE_CLAMP = 1.0;
    private static final double TIMEOUT_SEC = 4.0;
    private final Timer timer = new Timer();
    private final int selectedID;
    private final List<Integer> blueReefList = List.of(17, 18, 19, 20, 21, 22);
    private final List<Integer> redReefList = List.of(6, 7, 8, 9, 10, 11);
    private boolean RedAlliance;
    private boolean targetFound = false;

    public SelectiveDriveToPostOffset(SwerveSubsystem swerve, PhotonCamera camera, double forwardOffset, double sideOffset, int selectedID, LEDSubsystem ledSubsystem) {
        this.swerve = swerve;
        this.camera = camera;
        this.forwardOffset = forwardOffset;
        this.sideOffset = sideOffset;
        this.selectedID = selectedID;
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
        targetFound = false;
        System.out.println("SELECTIVE COMMAND RUNNING");
        System.out.println("Searching for Tag... " + selectedID);
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) {
            System.out.println("COMMAND NO TARGET");
            return;
        }
        ledSubsystem.breathProgres();

        System.out.println();
        System.out.println("///////////////////////////////////////");
        System.out.println();
        System.out.print("Tags in frame: ");
        for (PhotonTrackedTarget target : result.getTargets()) {
            System.out.print(target.getFiducialId() + " ");
        }
        System.out.println();

        PhotonTrackedTarget matchingTarget = null;
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == selectedID) {
                matchingTarget = target;
                break;
            }
        }
        if (matchingTarget == null) {
            System.out.println("Selected Tag " + selectedID + " not found in frame.");
            return;
        }
        System.out.println();
        targetFound = true;
        System.out.println("Matching target found with ID: " + matchingTarget.getFiducialId());
        boolean validTarget = RedAlliance ? redReefList.contains(matchingTarget.getFiducialId())
                                          : blueReefList.contains(matchingTarget.getFiducialId());

        System.out.println();
        if (!validTarget) {
            System.out.println("Valid target not found for alliance.");
            return;
        }
        double rawForwardDist = matchingTarget.getBestCameraToTarget().getX();
        double rawSideDist = matchingTarget.getBestCameraToTarget().getY();
        double forwardError = -rawForwardDist - forwardOffset;
        double forwardCmd = forwardPID.calculate(forwardError, 0.0);
        forwardCmd = MathUtil.clamp(forwardCmd, -FORWARD_CLAMP, FORWARD_CLAMP);
        double sideError = -rawSideDist - sideOffset;
        double sideCmd = sidePID.calculate(sideError, 0.0);
        sideCmd = MathUtil.clamp(sideCmd, -SIDE_CLAMP, SIDE_CLAMP);
        Logger.recordOutput("FWD PID", forwardCmd);
        Logger.recordOutput("SIDE PID", sideCmd);
        swerve.drive(forwardCmd, sideCmd, 0.0, false);
    }

    @Override
    public boolean isFinished() {
        boolean timedOut = timer.hasElapsed(TIMEOUT_SEC);
        boolean forwardOk = Math.abs(forwardPID.getPositionError()) < FORWARD_TOL;
        boolean sideOk = Math.abs(sidePID.getPositionError()) < SIDE_TOL;
        return timedOut || (targetFound && forwardOk && sideOk);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            ledSubsystem.BlinkBad();
        } else {
            ledSubsystem.BlinkGood();
        }
        swerve.drive(0.0, 0.0, 0.0, true);
    }
}
