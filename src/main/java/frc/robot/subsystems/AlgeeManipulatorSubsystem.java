// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
    config.closedLoop.p(0.1).i(0.0).d(0.0);
    ManipulatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.closedLoop.maxMotion
      .maxVelocity(3) //rotations per second
      .maxAcceleration(2) //rotations per second
      .allowedClosedLoopError(0.1);
    ManipulatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopController = ManipulatorMotor.getClosedLoopController();
    manipulatorEncoder = ManipulatorMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command L1SetpointPositionCommand() {
    
    return Commands.startEnd(
      ()-> closedLoopController.setReference(L1Setpoint, SparkBase.ControlType.kMAXMotionPositionControl),
      ()->StopMotor(),
      this)
      .until(()-> false);

  }

  public Command L2SetpointPositionCommand() {
    return Commands.startEnd(
      ()-> closedLoopController.setReference(L2Setpoint, SparkBase.ControlType.kMAXMotionPositionControl),
      ()->StopMotor(),
      this)
      .until(()-> false);

  }

  public Command L3SetpointPositionCommand() {
    
    return Commands.startEnd(
      ()-> closedLoopController.setReference(L3Setpoint, SparkBase.ControlType.kMAXMotionPositionControl),
      ()->StopMotor(),
      this)
      .until(()-> false);

  }
  public Command L4SetpointPositionCommand() {
    
    return Commands.startEnd(
      ()-> closedLoopController.setReference(L4Setpoint, SparkBase.ControlType.kMAXMotionPositionControl),
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
