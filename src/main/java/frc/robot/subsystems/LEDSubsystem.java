// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;



import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

public class LEDSubsystem extends SubsystemBase {
  public static AddressableLED LeftSide;
  public static AddressableLED RightSide;

  public static AddressableLEDBuffer LEDbuffer;
  public boolean isLEDrunning = false;
  private final LEDPattern m_rainbow;
  private final LEDPattern m_scrollingRainbow;
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  private static final LEDPattern base = LEDPattern.steps(Map.of(0, Color.kWhite, 0.5, Color.kBlue));
  private static final LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));
  public LEDSubsystem() {
    
    LeftSide = new AddressableLED(1);
    RightSide = new AddressableLED(2);

    LEDbuffer = new AddressableLEDBuffer(60);

    LeftSide.setLength(LEDbuffer.getLength());
    RightSide.setLength(LEDbuffer.getLength());

    LeftSide.setData(LEDbuffer);
    RightSide.setData(LEDbuffer);

    LeftSide.start();
    RightSide.start();

    m_rainbow = LEDPattern.rainbow(255, 128);
    m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    
    

    setDefaultCommand(runPattern(m_scrollingRainbow));

  }

  @Override
  public void periodic() {
    LeftSide.setData(LEDbuffer);
    RightSide.setData(LEDbuffer);
    if (!isLEDrunning) {
      
    } 
    // This method will be called once per scheduler run
  }

   public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(LEDbuffer));
  }

  public Command PulseCrusader() {
    return run(() -> pattern.applyTo(LEDbuffer));
  }

}

