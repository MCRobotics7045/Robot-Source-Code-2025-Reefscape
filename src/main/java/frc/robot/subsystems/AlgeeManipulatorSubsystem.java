// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants.AlgeeManipulatorConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static frc.robot.Constants.Constants.AlgeeManipulatorConstants.*;

import org.littletonrobotics.junction.Logger;

public class AlgeeManipulatorSubsystem extends SubsystemBase {
  public SparkMax ManipulatorMotor;
  public SparkClosedLoopController closedLoopController;
  public RelativeEncoder manipulatorEncoder;
  public SparkMaxConfig config;

  public AlgeeManipulatorSubsystem() {
    ManipulatorMotor = new SparkMax(Algee_Motor_ID, MotorType.kBrushless);
    
    config = new SparkMaxConfig();
    config.smartCurrentLimit(30).idleMode(IdleMode.kBrake).openLoopRampRate(0.1).inverted(false);
    ManipulatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.closedLoop.p(2.5).i(0.0).d(1);
    ManipulatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.closedLoop.maxMotion
      .maxVelocity(10000) //rotations per second
      .maxAcceleration(1000) //rotations per second
      .allowedClosedLoopError(1);
    ManipulatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopController = ManipulatorMotor.getClosedLoopController();
    manipulatorEncoder = ManipulatorMotor.getEncoder();

    


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Algee Arm", manipulatorEncoder.getPosition());
  }

  public Command dropOutCommand() {
    
    return Commands.startEnd(
      ()-> closedLoopController.setReference(-6, SparkBase.ControlType.kMAXMotionPositionControl),
      ()->StopMotor(),  
      this)
      .until(()-> false);

  }

  public Command HoldCommand() {
    return Commands.startEnd(
      ()-> closedLoopController.setReference(-4.6, SparkBase.ControlType.kMAXMotionPositionControl),
      ()->StopMotor(),
      this)
      .until(()-> false);

  }

  public Command StowPostion() {
    
    return Commands.startEnd(
      ()-> closedLoopController.setReference(-0.6, SparkBase.ControlType.kMAXMotionPositionControl),
      ()->StopMotor(),
      this)
      .until(()-> false);

  }


  public double getPosition() {
    return manipulatorEncoder.getPosition();
  }

  public void StopMotor() {
    ManipulatorMotor.stopMotor();
  }
}
