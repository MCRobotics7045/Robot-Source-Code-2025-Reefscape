// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgeeManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TalonFXElevatorSubsytem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GrabAlgaeFromReef extends Command {
  /** Creates a new GrabAlgaeFromReef. */
  private SwerveSubsystem swerve;
  private VisionSubsystem visionSubsystem;
  private AlgeeManipulatorSubsystem algeeManipulatorSubsystem;
  private TalonFXElevatorSubsytem elevatorSubsytem;
  private boolean isLevelTwo;
  private PhotonCamera camera;

  private final Timer timer = new Timer();
  private static final double TIMEOUT_SEC = 5.0;
  private final PIDController strafeController = new PIDController(1, 0,0);
  private final PIDController forawrdController = new PIDController(1, 0,0);
  private static final double FWD_TOL = 0.05;
  private static final double SIDE_TOL = 0.05;
  private static final double FWD_CLAMP = 0.2;
  private static final double SIDE_CLAMP = 0.2;
  
  List<Integer> blueReefList = java.util.List.of(17,18,19,20,21,22);
  List<Integer> redReefList = java.util.List.of(6,7,8,9,10,11);
  public boolean RedAlliance;

  public GrabAlgaeFromReef(SwerveSubsystem swerve, VisionSubsystem visionSubsystem,TalonFXElevatorSubsytem elevatorSubsytem,  AlgeeManipulatorSubsystem algeeManipulatorSubsystem, boolean isLevelTwo) {
    this.swerve = swerve;
    this.visionSubsystem = visionSubsystem;
    this.algeeManipulatorSubsystem = algeeManipulatorSubsystem;
    this.isLevelTwo = isLevelTwo;
    this.elevatorSubsytem = elevatorSubsytem;
    this.camera = visionSubsystem.bPCamera;
    addRequirements(swerve);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    RedAlliance = RobotContainer.IsRed();
    System.out.println("Algee Align Command Called");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (RedAlliance && result.hasTargets() ) { //RED
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      System.out.println("RED REEF TARGET FOUND ID:"+ bestTarget.getFiducialId());
    } else if (!RedAlliance && result.hasTargets() )  { //BLUE
      PhotonTrackedTarget bestTarget = result.getBestTarget();
    } else if (!RedAlliance && !result.hasTargets() ) {
          end(true);
          System.out.println("No Target Found Aborting...");
          return;
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
