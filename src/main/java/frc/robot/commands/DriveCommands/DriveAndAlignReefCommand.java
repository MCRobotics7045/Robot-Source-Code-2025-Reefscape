package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;


public class DriveAndAlignReefCommand extends Command {


    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;
    private final boolean approachLeft;

    private enum State {
        FIND_TAG,
        MACRO,
        MICRO,
        DONE
    }
    private State currentState = State.FIND_TAG;

    private Command pathCommand = null;
    private final Timer alignTimer = new Timer();
    private final double ALIGN_TIMEOUT = 2.0;

    private int TargetID;


    private final PIDController xPID = new PIDController(0.5, 0, 0);
    private final PIDController yPID = new PIDController(0.3, 0, 0);
    private final PIDController yawPID = new PIDController(0.02, 0, 0);

    // A small helper tolerance for x,y,yaw alignment
    private static final double X_TOL = 0.05;   // meters
    private static final double Y_TOL = 0.05;   // meters
    private static final double YAW_TOL = 2.0;  // degrees

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
                TargetID = vision.bestTargetID();
                if (bestTagPose == null) {
                    currentState = State.DONE;
                    break;
                    
                }
                Pose2d approachPose = computeApproachPose(bestTagPose, approachLeft);
                pathCommand = AutoBuilder.pathfindToPose(
                    approachPose,
                    new PathConstraints(3.0, 3.0, Math.toRadians(180), Math.toRadians(360))
                );
                if (pathCommand != null) {
                    pathCommand.initialize();
                }
                currentState = State.MACRO;
                break;

            case MACRO:
                if (pathCommand != null) {
                    pathCommand.execute();
                    if (pathCommand.isFinished()) {
                        pathCommand.end(false);
                        pathCommand = null;
                        alignTimer.reset();
                        alignTimer.start();
                        currentState = State.MICRO;
                    }
                } else {
                    currentState = State.MICRO;
                }
                break;

            case MICRO:
            if (!vision.seesTagID(TargetID)) {
                currentState = State.DONE;
                break;
            }

          
            Pose2d bestTagPoseAgain = vision.getBestReefAprilTagPose();
            if (bestTagPoseAgain == null) {
                currentState = State.DONE;
                break;
            }

            // desired final offset from the tag (e.g. small forward offset, no side offset)
            // or maybe a side offset if we want left vs. right
            Pose2d finalPose = computeApproachPose(bestTagPoseAgain, approachLeft);

            // get current robot pose
            Pose2d currentPose = swerve.getState().Pose;

            // measure error in x,y in field coordinates
            double errorX = finalPose.getX() - currentPose.getX();
            double errorY = finalPose.getY() - currentPose.getY();

            // measure heading error
            double currentHeading = currentPose.getRotation().getDegrees();
            double desiredHeading = finalPose.getRotation().getDegrees() + 180;
            double errorYaw = desiredHeading - currentHeading;

            // run PIDs
            double vx = xPID.calculate(0, errorX); // or xPID.calculate(currentPose.getX(), finalPose.getX())
            double vy = yPID.calculate(0, errorY);
            double omega = yawPID.calculate(0, errorYaw);

            // clamp speeds if needed
            vx = clamp(vx, 1.0);     // example clamp 1.0 m/s
            vy = clamp(vy, 1.0);
            omega = clamp(omega, 1.5); // example clamp 1.5 rad/s

            // drive field relative
            // swerve.drive(vx, vy, omega);

            // check done condition
            // boolean xAligned = Math.abs(errorX) < X_TOL;
            // boolean yAligned = Math.abs(errorY) < Y_TOL;
            // boolean yawAligned = Math.abs(errorYaw) < YAW_TOL;
            // boolean aligned = xAligned && yAligned && yawAligned;

            if (true || alignTimer.hasElapsed(ALIGN_TIMEOUT)) {
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
        if (pathCommand != null) {
            pathCommand.end(true);
        }
    }

    @Override
    public boolean isFinished() {
        return (currentState == State.DONE);
    }

    private Pose2d computeApproachPose(Pose2d bestTagPose, boolean approachLeftSide) {
        double forwardOffset = 0.5;
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
        Rotation2d reversedRot = origin.getRotation().plus(Rotation2d.fromDegrees(180));


        return new Pose2d(newX, newY, reversedRot);
    }


    private double clamp(double value, double maxMagnitude) {
        if (value > maxMagnitude) {
            return maxMagnitude;
        } else if (value < -maxMagnitude) {
            return -maxMagnitude;
        }
        return value;
    }
}
