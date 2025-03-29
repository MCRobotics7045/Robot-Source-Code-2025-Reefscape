 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Constants.SensorIOConstants.*;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.LimitSwitchConfig;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;


import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EndEffectorSubsystem;
public class SensorsIO extends SubsystemBase {
  
  public static Pigeon2 PigeonIMU;
  public DigitalInput ElevatorStowPostiontSensor;
  public DigitalInput CoralRampEnterSensor;
  public DigitalInput CoralEndEffectorEnterSensor;
  public static AnalogInput frontLeftUltrasonic;
  public static AnalogInput frontRightUltrasonic;
  public static AnalogInput rearLeftUltrasonic;
  public static AnalogInput rearRightUltrasonic;
  private static final int CALIBRATION_OFFSET = -18;
  private Counter counter;
  private int printedWarningCount = 5;
  private DigitalInput LiDAR;

  public SensorsIO() {
    PigeonIMU = new Pigeon2(Pigeon2Iid);
    LiDAR = new DigitalInput(LiDARSensorID);
    ElevatorStowPostiontSensor = new DigitalInput(StowPostiontSensorID);
    CoralEndEffectorEnterSensor = new DigitalInput(CoralEnterSensorID);
    CoralRampEnterSensor = new DigitalInput(CoralExitSensorID);
    counter = new Counter(LiDAR);
    counter.setMaxPeriod(1.0);
    counter.setSemiPeriodMode(true);
    counter.reset();

    // frontLeftUltrasonic = new AnalogInput(FrontLeft);
    // frontRightUltrasonic = new AnalogInput(FrontRight);
    // rearLeftUltrasonic = new AnalogInput(RearLeft);
    
    
  }

  @Override
  public void periodic() {
    
      SmartDashboard.putBoolean("Elevator Stowed", ElevatorStowPostiontSensor.get());
      SmartDashboard.putBoolean("Coral Ramp", CoralRampEnterSensor.get());
      SmartDashboard.putBoolean("Coral End Effector", CoralEndEffectorEnterSensor.get());
      Logger.recordOutput("Elevator Stowed", ElevatorStowPostiontSensor.get());
      Logger.recordOutput("Coral Ramp", CoralRampEnterSensor.get());
      Logger.recordOutput("Coral End Effector", CoralEndEffectorEnterSensor.get());
      SmartDashboard.putNumber("Pigeon 2 Yaw", PigeonIMU.getAngle());
      Logger.recordOutput("Pigeon 2 Yaw", PigeonIMU.getAngle());
      SmartDashboard.putNumber("LiDAR", getLiDARDistance());
      SmartDashboard.putBoolean("ReefDetected", isReefThreshold());
  }
   public BooleanSupplier getStowPositionSupplier() {
      return () -> !ElevatorStowPostiontSensor.get();
   }

   public boolean BoolStowPos() {
    return !ElevatorStowPostiontSensor.get();
   }
   public BooleanSupplier CoralRampEnterSensorTriggered() {
    return () -> !CoralRampEnterSensor.get();
   }

   public BooleanSupplier OppCoralRampEnterSensorTriggered() {
    return () -> CoralRampEnterSensor.get();
   }


   public BooleanSupplier CoralEndEffectorEnterSensorTriggered() {
    return () -> !CoralEndEffectorEnterSensor.get();
   }

   public Command ZeroPigeonIMU() {
    return Commands.runOnce(()-> PigeonIMU.setYaw(0));
   }


   public boolean isReefThreshold() {
      if (getLiDARDistance() > 70 && getLiDARDistance() < 79 ) {
        return true;
      } else {
        return false;
      }
   }

   public double getLiDARDistance() {
    double cm;
    if (counter.get() < 1) {
      if (printedWarningCount-- > 0) {
        System.out.println("LidarLitePWM: waiting for distance measurement");
      }
      return 0;
    }
    cm = (counter.getPeriod() * 1000000.0 / 10.0) + CALIBRATION_OFFSET;
    return cm;
  }

//    public static double ReadSensorinCM(AnalogInput Sensor) {
//     double Voltage = Sensor.getVoltage();
//     return Voltage * 100; //CM
//    }

//    public static boolean InRangeOfSensor(AnalogInput Sensor, double Distance) {
//     double CMREAD = ReadSensorinCM(Sensor);
//     return CMREAD < Distance;
//    }

//    public static String getObstaclePosition() {
//     boolean frontLeft = ReadSensorinCM(frontLeftUltrasonic) < Threashold;
//     boolean frontRight = ReadSensorinCM(frontRightUltrasonic) < Threashold;
//     boolean backLeft = ReadSensorinCM(rearLeftUltrasonic) < Threashold;
//     boolean backRight = ReadSensorinCM(rearRightUltrasonic) < Threashold;

//     if (frontLeft && frontRight) return "Front";
//     if (backLeft && backRight) return "Back";
//     if (frontLeft) return "Left Front";
//     if (frontRight) return "Right Front";
//     if (backLeft) return "Left Back";
//     if (backRight) return "Right Back";
    
//     return "None"; 
// }
}
