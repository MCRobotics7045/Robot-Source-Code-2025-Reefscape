// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndividualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.RobotContainer;
import frc.robot.subsystems.GripperSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RollerIn extends Command {
  private final GripperSubsystem Gripper;
  
public RollerIn(GripperSubsystem Gripper) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.Gripper = Gripper;
    addRequirements(Gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("COMMAND initialize");
    Gripper.RollerIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    System.out.println("COMMAND execute");
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Gripper.StopRoller();
    System.out.println("COMMAND isFinished FALSE and Command END CALLED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
