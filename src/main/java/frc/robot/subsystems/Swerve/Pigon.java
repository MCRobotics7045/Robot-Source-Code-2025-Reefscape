 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;
import static frc.robot.Constants.Constants.*;
public class Pigon extends SubsystemBase {
  
  public Pigeon2 PIGON;

  public Pigon() {
    PIGON = new Pigeon2(Pigeon2Iid);
  }

  // public Rotation2d getYawAsRotation2d() {
	// 	return Rotation2d.fromDegrees(PIGON.getYaw().getValue());
	// }

  // public double getYawFromPigeon() {
  //   // return PIGON.getYaw().getValue();
  // }

  public void zeroPigeon() {
    PIGON.setYaw(0);
  }

  public void overridePigeon(double NewAngle) {
    PIGON.setYaw(NewAngle);
  }

  
}
