// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.subsystems.Swerve.SensorsIO;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class AutoAlign extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final SensorsIO sensorsIO;
  private final boolean approachLeft;
  private final AprilTagFieldLayout fieldLayout;
  private enum State {
    SELECT_TAG,
    PATHING,
    MICRO,
    DONE
  }

  private double MAX_FWD_PATH = 3.0;
  private double MAX_ACL_PATH = 3.0;

  private State currentState = State.SELECT_TAG;
  private Command pathCommand = null;

  private final PIDController yawController = new PIDController(
    0.02, 
    0, 
    0
    );

  private final double YAW_TOL_DEG = 2.0;  

  private final Timer microTimer = new Timer();
  private static final double MICRO_TIMEOUT = 2.0;


  private static final List<Integer> RED_TAG_IDS   = List.of(6, 7, 8, 9, 10, 11);
  private static final List<Integer> BLUE_TAG_IDS  = List.of(17, 18, 19, 20, 21, 22);

  private int Tag_id; 

  private static final double FORWARD_OFFSET = 0.3;
  private static final double SIDE_OFFSET    = 0.4;


  /** Creates a new AutoAlign. */
  public AutoAlign(SwerveSubsystem swerveSubsystem, boolean approachLeftSide, SensorsIO sensorsIO, AprilTagFieldLayout fieldLayout) {
    this.swerveSubsystem = swerveSubsystem;
    this.approachLeft = approachLeftSide;
    this.sensorsIO = sensorsIO;
    this.fieldLayout = fieldLayout;

    addRequirements(swerveSubsystem);
  }
  @Override
  public void initialize() {currentState = State.SELECT_TAG;}

  
  @Override
  public void execute() { switch (currentState) {
    case SELECT_TAG:
      Pose2d closestPylonPick = findClosestReefTag(approachLeft);
      if (closestPylonPick == null) {
        currentState = State.DONE;
        break;
      }
      pathCommand = AutoBuilder.pathfindToPose(
        closestPylonPick,
        new PathConstraints(
        MAX_FWD_PATH,
        MAX_ACL_PATH,
        Units.degreesToRadians(180),
        Units.degreesToRadians(360)
      )
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
          microTimer.reset();
          microTimer.start();
          currentState = State.MICRO;
       }
      } else {
        currentState = State.MICRO;
      }
      break;

    case MICRO:
      double currentYawDeg = swerveSubsystem.getState().Pose.getRotation().getDegrees();
      double desiredYawDeg = getTagYaw(Tag_id).getRotation().getDegrees();
      double yawError = desiredYawDeg - currentYawDeg;
      double turnCmd = yawController.calculate(0, yawError);
      turnCmd = Math.max(Math.min(turnCmd, 0.4), -0.4);
      swerveSubsystem.drive(0, 0, turnCmd, true);
      boolean yawAligned = Math.abs(yawError) < YAW_TOL_DEG;
      if (yawAligned || microTimer.hasElapsed(MICRO_TIMEOUT)) {
        currentState = State.DONE;
      }           
      //ultrasonic check aswell
      break;

      case DONE:
      default:
          break;
  }
}
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(0, 0, 0, true);
    if (pathCommand != null) {
        pathCommand.end(true);
    }
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currentState == State.DONE);
  }

  private Pose2d findClosestReefTag(boolean approachLeftSide) {
        boolean isRed = RobotContainer.IsRed();
        List<Integer> possibleTags = isRed ? RED_TAG_IDS : BLUE_TAG_IDS;

        Pose2d robotPose = swerveSubsystem.getState().Pose;
        Pose2d bestPose = null;
        double bestDistance = Double.MAX_VALUE;

        for (int tagID : possibleTags) {
           
            Optional<Pose2d> maybePose = getTagPose2d(tagID);
            if (maybePose.isEmpty()) continue;

            Pose2d tagPose = maybePose.get();
            double dist = robotPose.getTranslation().getDistance(tagPose.getTranslation());
            if (dist < bestDistance) {
                bestDistance = dist;
                bestPose = tagPose;
                Tag_id = tagID;
            }
        }
        return bestPose; // can be null if no tags found
    }


    private Optional<Pose2d> getTagPose2d(int tagID) {

      var maybeTagPose3d = fieldLayout.getTagPose(tagID);
      if (maybeTagPose3d.isEmpty()) {
          return Optional.empty();
      }

        Pose2d TagPose = maybeTagPose3d.get().toPose2d();

        Pose2d offsetedPose = offsetPose(TagPose,approachLeft,FORWARD_OFFSET,SIDE_OFFSET);

        return Optional.of(offsetedPose);

    }

    private Pose2d getTagYaw(int tagID)  {

     
        Pose2d TagPose = fieldLayout.getTagPose(tagID).get().toPose2d();

        Pose2d offsetedPose = offsetPose(TagPose,approachLeft,FORWARD_OFFSET,SIDE_OFFSET);

        return offsetedPose;

    }

    private Pose2d offsetPose(Pose2d baseTagPose, boolean approachLeftSide, double forwardMeters, double sideMeters) {
      double baseDeg = baseTagPose.getRotation().getDegrees();
      double side = approachLeftSide ? sideMeters : -sideMeters;

      double xForward = forwardMeters * Math.cos(Math.toRadians(baseDeg));
      double yForward = forwardMeters * Math.sin(Math.toRadians(baseDeg));
      double xLateral = side * Math.cos(Math.toRadians(baseDeg + 90));
      double yLateral = side * Math.sin(Math.toRadians(baseDeg + 90));

      double newX = baseTagPose.getX() + xForward + xLateral;
      double newY = baseTagPose.getY() + yForward + yLateral;

      // rotate 180 deg so we face the reef
      Rotation2d reversedRot = baseTagPose.getRotation().plus(Rotation2d.fromDegrees(180));

      return new Pose2d(newX, newY, reversedRot);
  }
}
