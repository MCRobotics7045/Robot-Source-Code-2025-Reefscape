// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;



import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

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

  private static final LEDPattern SlowBase = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kDarkOrange));
  private static final LEDPattern SLOW_PATTERN = SlowBase.scrollAtRelativeSpeed(Percent.per(Second).of(10));
  
  private static final LEDPattern BreathBase = LEDPattern.gradient(GradientType.kDiscontinuous,Color.kRed, Color.kBlue);
  private static final LEDPattern BREATH_LED_PATTERN = BreathBase.breathe(Seconds.of(1));

  private static final LEDPattern Blink = LEDPattern.solid(Color.kRed);
  private static final LEDPattern BlinkBad = Blink.blink(Seconds.of(1));

  private static final LEDPattern BlinkG = LEDPattern.solid(Color.kGreen);
  private static final LEDPattern BlinkGood = BlinkG.blink(Seconds.of(1));

  
  
  public LEDSubsystem() {
    
    LeftSide = new AddressableLED(7);
    // RightSide = new AddressableLED(8);

    LEDbuffer = new AddressableLEDBuffer(60);

    LeftSide.setLength(LEDbuffer.getLength());
    // RightSide.setLength(LEDbuffer.getLength());

    LeftSide.setData(LEDbuffer);
    // RightSide.setData(LEDbuffer);

    LeftSide.start();
    // RightSide.start();

    m_rainbow = LEDPattern.rainbow(255, 128);
    m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    
    

    setDefaultCommand(PulseCrusader());

  }

  @Override
  public void periodic() {
    LeftSide.setData(LEDbuffer);
    // RightSide.setData(LEDbuffer);

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

  public Command SlowMode() {
    return run(() -> SLOW_PATTERN.applyTo(LEDbuffer));
  }


  public Command breathProgres() {
    return run(() -> BREATH_LED_PATTERN.applyTo(LEDbuffer));
  }



  public Command BlinkBadC() {
    return run(() -> BlinkBad());
  }

  public Command BlinkGoodC() {
    return run(() -> BlinkGood());
  }
  public void BlinkBad() {
    BlinkBad.applyTo(LEDbuffer);
  }

  public void BlinkGood() {
    BlinkGood.applyTo(LEDbuffer);
  }

  public Command ChangeColor(int colorID) {
    return runOnce(() -> {
      Color selectedColor;

      switch (colorID) {
        case 1:
          selectedColor = Color.kRed;
          break;
        case 2:
          selectedColor = Color.kOrange;
          break;
        case 3:
          selectedColor = Color.kYellow;
          break;
        case 4:
          selectedColor = Color.kGreen;
          break;
        case 5:
          selectedColor = Color.kBlue;
          break;
        case 6:
          selectedColor = Color.kPurple;
          break;
        case 7:
          selectedColor = Color.kPink;
          break;
        case 8:
          selectedColor = Color.kCyan;
          break;
        case 9:
          selectedColor = Color.kLime;
          break;
        case 10:
          selectedColor = Color.kDarkBlue;
          break; 
        default:
          selectedColor = Color.kWhite; // Default to white if invalid number
      }

      for (int i = 0; i < LEDbuffer.getLength(); i++) {
        LEDbuffer.setLED(i, selectedColor);
      }
    });
  }
}

