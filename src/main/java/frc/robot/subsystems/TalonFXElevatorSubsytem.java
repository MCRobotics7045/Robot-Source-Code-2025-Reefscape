// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.RobotContainer.SENSORS;

import java.io.Serial;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TalonFXElevatorSubsytem extends SubsystemBase {
  /** Creates a new TalonFXElevatorSubsytem. */
  public final TalonFX ElevatorMotor;
  private final MotionMagicVoltage m_request;
  
  public TalonFXElevatorSubsytem() {
    ElevatorMotor = new TalonFX(20, "rio");
     m_request = new MotionMagicVoltage(0);
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    slot0Configs.kP = 2; 
    slot0Configs.kI = 0;
    slot0Configs.kD = 0; 

    motionMagicConfigs.MotionMagicCruiseVelocity = 90; 
    motionMagicConfigs.MotionMagicAcceleration = 100; 


    ElevatorMotor.getConfigurator().apply(talonFXConfigs);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (SENSORS.BoolStowPos()) {
      ElevatorMotor.setPosition(0);
    }

    Logger.recordOutput("Elevator Height", ElevatorMotor.get());
  }
  
  public Command ReefSetpointPositionCommand(double SetPoint) {
    return Commands.startEnd(
      ()-> ElevatorMotor.setControl(m_request.withPosition(SetPoint)),
      ()->System.out.println("Elevator Requested at Height:"+ SetPoint),
      this );
    //  .andThen(()-> System.out.println("Elevator Requested at Height:"+ SetPoint));
  }

  public Command DropElevator() {
    return Commands.runOnce(()-> ElevatorMotor.setControl(m_request.withPosition(0)))
    .until(SENSORS.getStowPositionSupplier());
    // .andThen(()->StopMotor());
    //.andThen(()->ElevatorMotor.setPosition(0));
    // .andThen(()-> RobotContainer.createRumbleCommand(1, 1, 0.3));
  }

  public Command SpoolCommand() {
    return Commands.runOnce(()-> ElevatorMotor.set(-1));
  }

  public Command UnspoolCommand() {
    return Commands.runOnce(()-> ElevatorMotor.set(1));
  }
  public void StopMotor() {
    ElevatorMotor.stopMotor();
  }
}
