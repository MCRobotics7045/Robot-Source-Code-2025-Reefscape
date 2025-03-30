// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import static frc.robot.Constants.Constants.InputConstants.xboxLeftStickDeadband;
import static frc.robot.Constants.Constants.InputConstants.xboxRightStickDeadband;
import static frc.robot.Constants.Constants.SwerveConstants.SlewRate;
import static frc.robot.Constants.Constants.SwerveConstants.angularSpeed;
import static frc.robot.Constants.Constants.SwerveConstants.speedMultiplier;
import java.util.LinkedList;
import java.util.Queue;

import org.littletonrobotics.junction.Logger;

import static java.lang.Math.cos;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class DefaultDrive extends Command {
  private final int FILTER_SIZE = 20;
  private final Queue<Double> xQueue = new LinkedList<>();
  private double sumX = 0.0;
  public double MaxSpeed = 1;
  private CommandXboxController XBOX;
  private SwerveSubsystem SWERVE;
  private double xVelocity;
  private double yVelocity;
  private double rotationalVelocity;

  SlewRateLimiter xVelocityFilter = new SlewRateLimiter(7);
  SlewRateLimiter yVelocityFilter = new SlewRateLimiter(7);
  SlewRateLimiter xRotateFilter = new SlewRateLimiter(10);


  public DefaultDrive(CommandXboxController  XBOX, SwerveSubsystem SWERVE) {
    this.SWERVE = SWERVE;
    this.XBOX = XBOX;
    addRequirements(SWERVE);
    
  }

 
  @Override
  public void execute() {
    // double InputX = XBOX.getLeftX();
    // double InputY = XBOX.getLeftY();
    // double InputZ = XBOX.getRightY();
    // SmartDashboard.putNumber("Input X", InputX);
    // SmartDashboard.putNumber("Input Y", InputY);
    // SmartDashboard.putNumber("Input Z", InputZ);
    // InputX = MathUtil.applyDeadband(InputX, .15);
    // InputY = MathUtil.applyDeadband(InputY, .15);
    // InputZ = MathUtil.applyDeadband(InputZ, .15);

    // InputX = InputX * InputX;
    // InputY = InputY * InputY;
    // double forwardDirection = (RobotContainer.IsRed() ? 1.0 : -1.0);
    // double inputDir = Math.atan2(InputY, InputX);
    // double inputMagnitude = Math.hypot(InputX, InputY);
		// double xVelocity = xVelocityFilter.calculate(cos(inputDir) * inputMagnitude * MaxSpeed * forwardDirection * speedMultiplier);
    // double yVelocity = yVelocityFilter.calculate(sin(inputDir) * inputMagnitude * MaxSpeed * forwardDirection * speedMultiplier);
    // double rotationalVelocity = (InputZ * angularSpeed );

    // SWERVE.drive(yVelocity, xVelocity, rotationalVelocity, true);

    double InputX = MathUtil.applyDeadband(-XBOX.getLeftX(), .15) * SWERVE.MaxSpeed;
    double InputY = MathUtil.applyDeadband(XBOX.getLeftY(), .15) * SWERVE.MaxSpeed;
    double InputZ = MathUtil.applyDeadband(-XBOX.getRightX(), .15) * SWERVE.MaxRotSpeed;


    // double InputX = Math.pow(MathUtil.applyDeadband(XBOX.getLeftX(), .15),2);
    // double InputY = Math.pow(MathUtil.applyDeadband(XBOX.getLeftY(), .15),2);
    // double InputZ = Math.pow(MathUtil.applyDeadband(XBOX.getRightX(), .12),2);
    xVelocity = xVelocityFilter.calculate(InputX);
    yVelocity = yVelocityFilter.calculate(InputY);
    rotationalVelocity = xRotateFilter.calculate(InputZ);
    
    Logger.recordOutput("Input X Out", xVelocity);
    Logger.recordOutput("Input Y Out", yVelocity);
    Logger.recordOutput("Input Z Out", InputZ);

    SWERVE.drive(xVelocity, yVelocity, rotationalVelocity, true);
    
    
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // public double movingAverageFilter(double rawX) {
  //   xQueue.add(rawX);
  //   sumX += rawX;
    
  //   if(xQueue.size() > FILTER_SIZE) {
  //       sumX -= xQueue.remove();
  //   }
  //   return sumX / xQueue.size();
  // }
}






