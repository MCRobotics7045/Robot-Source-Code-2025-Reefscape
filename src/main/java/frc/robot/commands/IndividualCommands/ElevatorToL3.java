// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndividualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

//****************************************************************** 
//****************************************************************** 
//DO NOT USE THIS COMMAND RAW. IT WILL CAUSE THE CORAL TO JAM
//USE THE SEQUENTIAL COMMAND INSTEAD
//****************************************************************** 
//****************************************************************** 

public class ElevatorToL3 extends Command {
  /** Creates a new ElevatorToL3. */
  ElevatorSubsystem elevator = new ElevatorSubsystem();

  public ElevatorToL3(ElevatorSubsystem elevator ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setPointL3();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setPointL3();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.StopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // how should i check for L2 postion. maybe encoder or check revolutions 
    if (elevator.StowPostionSwitch() == false && elevator.MaxHeightSwitch() == false) {
      return true;
    } else {
      return false;
    }
  }
}
