// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.SwerveConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
// import static frc.robot.Constants.Constants.SwerveConstants.*;
import static frc.robot.Constants.Constants.InputConstants.*;

public class AlignInFrontofTagCommand extends Command {
  /** Creates a new AlignInFrontofTagCommand. */
  private final SwerveSubsystem SWERVE;
  private final VisionSubsystem VISION;
  private final PhotonCamera posCamera;
  private final XboxController XBOX;
  private final PIDController rotationController;
  private int targetID;
    public AlignInFrontofTagCommand(SwerveSubsystem SWERVE, VisionSubsystem VISION, PhotonCamera posCamera, XboxController XBOX ) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.SWERVE = SWERVE;
      this.VISION = VISION;
      this.posCamera = posCamera;
      this.XBOX = XBOX;
  
      // int targetID = 0;
      rotationController = new PIDController(0, 0, 0); // I HATE PIDS BUT I HAVE TO USE THEM SO MY GOOD FRIEND CHAT GPT WILL 95% BE DOING THIS WHEN I HAVE TO CHANGE TO TUNE THEM CAUSE TUNING PIDS SUCK LIKE PID WHO NAMED THAT BRO
      rotationController.enableContinuousInput(-Math.PI, Math.PI);
      addRequirements(SWERVE, VISION);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      //Pull April Tag Data And Find Calulation
      
  
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      //Drive And Execute the command 
  
      Pose2d robotPose = SWERVE.getPose();
      
      var results = posCamera.getLatestResult(); 
      if (results.hasTargets() == false) {
        System.out.print("WARNING: No Tag Detected or Found. If tag is visble jack is not at fault");
        this.cancel(); 
        return;
      } else {
        PhotonTrackedTarget target = results.getBestTarget();
        targetID = target.getFiducialId();
      // posCamera.takeInputSnapshot();  //Debug Tool!
      Pose2d tagPose = VISION.getAprilTagPose2d(targetID);
      Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose, tagPose);

      double CalcualtedRotationalVelocity = rotationController.calculate(robotPose.getRotation().getRadians(), targetYaw.getRadians()); //I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS 

      double InputX = MathUtil.applyDeadband(XBOX.getLeftX(), xboxLeftStickDeadband) * SwerveConstants.MaxSpeed;
      double InputY = MathUtil.applyDeadband(XBOX.getLeftY(), xboxLeftStickDeadband) * SwerveConstants.MaxSpeed;

      SWERVE.drive(InputX, InputY, CalcualtedRotationalVelocity);
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("AlignCommandFinished. If it didnt work oh well");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    Pose2d targetPose = VISION.getAprilTagPose2d(targetID);
    if (targetPose == null) {
        System.out.println("No tag detected. Ending command.");
        return true; 
    }
    return Math.abs(rotationController.getPositionError()) < 0.05; 

  }
}
