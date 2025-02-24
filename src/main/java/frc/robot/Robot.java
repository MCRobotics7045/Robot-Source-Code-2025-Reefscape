// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.subsystems.VisionSubsystem;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  // private VisionSubsystem vision = new VisionSubsystem();
  private RobotContainer m_robotContainer;
  Optional<Alliance> ally;

  @Override
  public void robotInit() {
    //Has To stay on Top
    m_robotContainer = new RobotContainer();
    //Has To stay on Top
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



    // Logger.recordOutput("Pose/BotPose", RobotContainer.SWERVE.getPose());



    ally =  DriverStation.getAlliance();
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Match Round", DriverStation.getMatchNumber());
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
          SmartDashboard.putString("Alliance", "Red");
      }
      if (ally.get() == Alliance.Blue) {
          SmartDashboard.putString("Alliance", "Blue");
      }
    } else {
      SmartDashboard.putString("Alliance", "Cant Find");
    }
  
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

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
    // vision.simulationPeriodic();
  }
}
