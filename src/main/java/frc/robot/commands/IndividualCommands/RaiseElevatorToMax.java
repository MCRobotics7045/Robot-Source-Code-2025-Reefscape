// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndividualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RaiseElevatorToMax extends Command {
  /** Creates a new DropElevatorToStow. */
  private final ElevatorSubsystem elevatorSubsystem;
  public RaiseElevatorToMax(ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Elastic.Notification notification = new Notification(NotificationLevel.INFO, "RaiseElevatorToMax Command", "RaiseElevatorToMax Command has been Called");
    Elastic.sendNotification(notification);
    elevatorSubsystem.RaiseMax();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.RaiseMax();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.StopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (elevatorSubsystem.StowPostionSwitch() == false && elevatorSubsystem.MaxHeightSwitch()) {
      return true;
    } else {
      return false;
    }
  }
}
