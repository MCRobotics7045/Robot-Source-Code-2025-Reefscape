// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.IndividualCommands.DropElevatorToStow;
import frc.robot.commands.IndividualCommands.IntakeCoralTillStopped;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCoralFromFeedStation extends ParallelCommandGroup {
  /** Creates a new IntakeCoralFromFeedStation. */
  public IntakeCoralFromFeedStation(ElevatorSubsystem elevatorSubsystem, GripperSubsystem gripperSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeCoralTillStopped(gripperSubsystem),
      new DropElevatorToStow(elevatorSubsystem)
    );
  }
}
