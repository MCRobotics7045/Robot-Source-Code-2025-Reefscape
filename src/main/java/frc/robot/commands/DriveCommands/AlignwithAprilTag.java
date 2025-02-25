// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;



import org.photonvision.PhotonCamera;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignwithAprilTag extends Command {
  /** Creates a new AlignwithAprilTag. */
  private PhotonCamera camera;
  private final SwerveSubsystem swerveSubsystem;
  private final int LookingTagId;
  private final PIDController turnController = new PIDController(0, 0, 0);
  private final PIDController driveController = new PIDController(0, 0, 0);
  private boolean IsAligned;
  public AlignwithAprilTag(PhotonCamera camera,SwerveSubsystem swerveSubsystem, int LookingTagId) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.camera = camera;
    this.swerveSubsystem = swerveSubsystem;
    this.LookingTagId = LookingTagId;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IsAligned = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      int targetID = target.getFiducialId();
      if (targetID == LookingTagId) {
        double targetYaw = target.getYaw();
        double distance = target.getBestCameraToTarget().getZ();
        double turnSpeed = turnController.calculate(targetYaw, 0);
        double forwardSpeed = driveController.calculate(distance, 1.0);

        swerveSubsystem.drive(forwardSpeed, 0, -turnSpeed);

        if (Math.abs(targetYaw) < 1.0 && Math.abs(distance - 1.0) < 0.05) {
          IsAligned = true;
        }

      } else {
        System.out.println("Warning AprilTag Deteced but it isnt The Correct ID");
      }
    } else {
      System.out.println("Warning no results found");
      swerveSubsystem.drive(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return IsAligned;
  }
}
