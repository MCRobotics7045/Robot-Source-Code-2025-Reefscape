// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TunePIDDrive extends Command {
  /** Creates a new TunePIDDrive. */
  private static final double X_VELOCITY = 1.0; // Forward speed (m/s)
  private static final double Y_VELOCITY = 0.0; // Strafe speed (m/s)
  private static final double ROTATIONAL_VELOCITY = 0.0; // No rotation
  private final SwerveSubsystem Swerve;
  public TunePIDDrive(SwerveSubsystem Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.Swerve = Swerve;
    addRequirements(Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveTestCommand initialized.");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Swerve.drive(X_VELOCITY, Y_VELOCITY, ROTATIONAL_VELOCITY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Swerve.drive(0.0, 0.0, 0.0);
    System.out.println("DriveTestCommand ended. Interrupted: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
