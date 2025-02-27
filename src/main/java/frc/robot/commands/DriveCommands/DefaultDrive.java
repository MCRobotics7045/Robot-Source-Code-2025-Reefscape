// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import static frc.robot.Constants.Constants.SwerveConstants.*;
import static frc.robot.Constants.Constants.InputConstants.*;

public class DefaultDrive extends Command {

  private CommandXboxController XBOX;
  private SwerveSubsystem SWERVE;
  private double xVelocity;
  private double yVelocity;
  private double rotationalVelocity;

  SlewRateLimiter xVelocityFilter = new SlewRateLimiter(SlewRate);
  SlewRateLimiter yVelocityFilter = new SlewRateLimiter(SlewRate);
  SlewRateLimiter xRotateFilter = new SlewRateLimiter(SlewRate);


  public DefaultDrive(CommandXboxController  XBOX, SwerveSubsystem SWERVE) {
    this.SWERVE = SWERVE;
    this.XBOX = XBOX;
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

    double InputX = MathUtil.applyDeadband(XBOX.getLeftX(), xboxLeftStickDeadband) * SwerveConstants.MaxSpeed;
    double InputY = MathUtil.applyDeadband(XBOX.getLeftY(), xboxLeftStickDeadband) * SwerveConstants.MaxSpeed;
    double InputZ = MathUtil.applyDeadband(XBOX.getRightX(), xboxRightStickDeadband) * SwerveConstants.MaxRotationSpeed;
    yVelocity = InputY;
    xVelocity = InputX;
    rotationalVelocity = InputZ;
    xVelocity = xVelocityFilter.calculate(InputX);
    yVelocity = yVelocityFilter.calculate(InputY);
    rotationalVelocity = xRotateFilter.calculate(InputZ);
    SWERVE.drive(yVelocity, xVelocity, rotationalVelocity);
    
    
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
