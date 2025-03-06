package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveAndAlignReefCommand extends Command {
//JAMES THIS IS TOTTALY MY CODE AND NOT TEAM 7414 RETROROBOTICS CODE THAT MAY OR MAY NOT WORK 
//I GAVE UP HALF WAY THROUGH MAKING IT SO I RESORTED TO BECOMEING A GITHUB PIRATE  
//THIS ISNT JUST A STRAIGHT COPY BUT JUST A ADAPTATION OF IT

    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;
    private final boolean approachLeft;

    private enum State {
        FIND_TAG,
        PATHING,
        FINAL_ALIGN,
        DONE
    }
    private State currentState = State.FIND_TAG;

    private Command pathCommand = null;
    private final Timer alignTimer = new Timer();
    private final double ALIGN_TIMEOUT = 2.0;

    public DriveAndAlignReefCommand(SwerveSubsystem swerve, VisionSubsystem vision, boolean approachLeft) {
        this.swerve = swerve;
        this.vision = vision;
        this.approachLeft = approachLeft;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        currentState = State.FIND_TAG;
    }

    @Override
    public void execute() {
        switch (currentState) {

            case FIND_TAG:
                Pose2d bestTagPose = vision.getBestReefAprilTagPose();
                if (bestTagPose == null) {
                    bestTagPose = new Pose2d(3.0, 3.0, swerve.getPose().getRotation());
                }
                Pose2d approachPose = computeApproachPose(bestTagPose, approachLeft);
                pathCommand = AutoBuilder.pathfindToPose(
                    approachPose,
                    new PathConstraints(3.0, 3.0, Math.toRadians(360), Math.toRadians(720))
                );
                if (pathCommand != null) {
                    pathCommand.initialize();
                }
                currentState = State.PATHING;
                break;

            case PATHING:
                if (pathCommand != null) {
                    pathCommand.execute();
                    if (pathCommand.isFinished()) {
                        pathCommand.end(false);
                        pathCommand = null;
                        alignTimer.reset();
                        alignTimer.start();
                        currentState = State.FINAL_ALIGN;
                    }
                } else {
                    currentState = State.FINAL_ALIGN;
                }
                break;

            case FINAL_ALIGN:
                double yawError = vision.getBestTagYaw();
                double turnCmd = 0.0;
                double kP = 0.01;
                turnCmd = kP * yawError;
                if (turnCmd > 0.3) turnCmd = 0.3;
                if (turnCmd < -0.3) turnCmd = -0.3;
                swerve.drive(0.0, 0.0, turnCmd);
                boolean aligned = (Math.abs(yawError) < 2.0);
                if (aligned || alignTimer.hasElapsed(ALIGN_TIMEOUT)) {
                    currentState = State.DONE;
                }
                break;

            case DONE:
            default:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0);
        if (pathCommand != null) {
            pathCommand.end(true);
        }
    }

    @Override
    public boolean isFinished() {
        return (currentState == State.DONE);
    }

    private Pose2d computeApproachPose(Pose2d bestTagPose, boolean approachLeftSide) {
        double forwardOffset = -0.5;
        double sideOffset = approachLeftSide ? 0.3 : -0.3;
        return offsetPose(bestTagPose, bestTagPose.getRotation().getDegrees(), forwardOffset, sideOffset);
    }

    private Pose2d offsetPose(Pose2d origin, double baseDegrees, double forward, double lateral) {
        double xForward = forward * Math.cos(Math.toRadians(baseDegrees));
        double yForward = forward * Math.sin(Math.toRadians(baseDegrees));
        double xLateral = lateral * Math.cos(Math.toRadians(baseDegrees + 90));
        double yLateral = lateral * Math.sin(Math.toRadians(baseDegrees + 90));
        double newX = origin.getX() + xForward + xLateral;
        double newY = origin.getY() + yForward + yLateral;
        return new Pose2d(newX, newY, origin.getRotation());
    }
}
