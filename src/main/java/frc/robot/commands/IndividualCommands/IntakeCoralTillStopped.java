// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndividualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GripperSubsystem;


public class IntakeCoralTillStopped extends Command {
  /** Creates a new IntakeCoralTillStopped. */
  private final GripperSubsystem gripperSubsystem;  

  public IntakeCoralTillStopped(GripperSubsystem gripperSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripperSubsystem = gripperSubsystem;
    addRequirements(gripperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gripperSubsystem.RollerIntakeCoral();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gripperSubsystem.RollerIntakeCoral();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripperSubsystem.StopRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gripperSubsystem.CoralExitSensorTriggered() && gripperSubsystem.CoralEnterSensorTriggered();
  }
}
