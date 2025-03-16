// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import static frc.robot.Constants.Constants.InputConstants.xboxLeftStickDeadband;
import static frc.robot.Constants.Constants.InputConstants.xboxRightStickDeadband;
import static frc.robot.Constants.Constants.SwerveConstants.SlewRate;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class DriveWithJoystick extends Command {
  private final int FILTER_SIZE = 5;
  private final Queue<Double> xQueue = new LinkedList<>();
  private double sumX = 0.0;

  private CommandJoystick joystick;
  private SwerveSubsystem SWERVE;
  private double xVelocity;
  private double yVelocity;
  private double rotationalVelocity;

  SlewRateLimiter xVelocityFilter = new SlewRateLimiter(SlewRate);
  SlewRateLimiter yVelocityFilter = new SlewRateLimiter(SlewRate);
  SlewRateLimiter xRotateFilter = new SlewRateLimiter(SlewRate);


  public DriveWithJoystick(CommandJoystick joystick, SwerveSubsystem SWERVE) {
    this.SWERVE = SWERVE;
    this.joystick = joystick;
    addRequirements(SWERVE);
    
  }

 
  @Override
  public void execute() {
    // double InputX = XBOX.getLeftX();
    // double InputY = XBOX.getLeftY();
    // double InputZ = XBOX.getRightX();
    // InputX = MathUtil.applyDeadband(InputX, .1);
    // InputY = MathUtil.applyDeadband(InputY, .1);
    // double forwardDirection = (RobotContainer.IsRed() ? 1.0 : -1.0);
    // double inputDir = Math.atan2(InputY, InputX);
    // double inputMagnitude = Math.hypot(InputX, InputY);
		// double xVelocity = xVelocityFilter.calculate(cos(inputDir) * inputMagnitude * MaxSpeed * forwardDirection * SWERVE.SpeedMultipler);
    // double yVelocity = yVelocityFilter.calculate(sin(inputDir) * inputMagnitude * MaxSpeed * forwardDirection * SWERVE.SpeedMultipler);
    // double rotationalVelocity = (InputZ * angularSpeed );
    // SWERVE.drive(yVelocity, xVelocity, rotationalVelocity);

    double InputX = MathUtil.applyDeadband(joystick.getX(), xboxLeftStickDeadband) * SwerveConstants.MaxSpeed;
    double InputY = MathUtil.applyDeadband(joystick.getY(), xboxLeftStickDeadband) * SwerveConstants.MaxSpeed;
    double InputZ = MathUtil.applyDeadband(joystick.getZ(), xboxRightStickDeadband) * SwerveConstants.MaxRotationSpeed;
    yVelocity = movingAverageFilter(InputY);
    xVelocity = movingAverageFilter(InputX);
    rotationalVelocity = movingAverageFilter(InputZ);
    xVelocity = xVelocityFilter.calculate(InputX);
    yVelocity = yVelocityFilter.calculate(InputY);
    rotationalVelocity = xRotateFilter.calculate(InputZ);
    SWERVE.drive(yVelocity, xVelocity, -rotationalVelocity, true);
    
    
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public double movingAverageFilter(double rawX) {
    xQueue.add(rawX);
    sumX += rawX;
    
    if(xQueue.size() > FILTER_SIZE) {
        sumX -= xQueue.remove();
    }
    return sumX / xQueue.size();
  }
}






