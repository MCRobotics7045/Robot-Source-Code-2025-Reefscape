// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private VisionSubsystem vision ;
  private RobotContainer m_robotContainer;
  Optional<Alliance> ally;

  @Override
  public void robotInit() {
    //Has To stay on Top
    m_robotContainer = new RobotContainer();
    //Has To stay on Top

    vision = RobotContainer.VISION;

    SmartDashboard.putString("Event Name", DriverStation.getEventName());
    SmartDashboard.putString("Alliance", "Cant Find");
    Logger.recordMetadata("ProjectName", "MyProject");
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); 
      Logger.addDataReceiver(new NT4Publisher()); 
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }
  
    Logger.start(); 
  }


  @Override
  public void robotPeriodic() {
    //Has To stay on Top
    //Has To stay on Top
    CommandScheduler.getInstance().run(); 
    //Has To stay on Top
    //Has To stay on Top



    Logger.recordOutput("Pose/BotPose", RobotContainer.SWERVE.getState().Pose);



 
  
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    RobotContainer.LED.PulseCrusader().schedule();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    // vision..s();
  }
}
