// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class LEDSubsystem extends SubsystemBase {

  AddressableLED m_led = new AddressableLED(0);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(120);

  /** Creates a new LEDController. */
  public LEDSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void clearLEDs(){
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
    m_led.setData(m_ledBuffer);
  }

  public Command setColor(Color color) {
    return this.runOnce(() -> {
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
      }
      m_led.setData(m_ledBuffer);
    });
  }

  public Command strobeLED(Color color, double period) {
    return setColor(color).andThen(
        new WaitCommand(period / 2)).andThen(
            setColor(Color.kBlack))
        .andThen(
            new WaitCommand(period / 2))
        .repeatedly();
  }

  double fadeElapsedTime=0;
  public Command fadeIn(Color endColor, double time){
      return runOnce(()->{
        fadeElapsedTime = 0;
      }).andThen(run(()->{
        double fadePercent = fadeElapsedTime / time;
        Color color=new Color(endColor.red*fadePercent, endColor.green*fadePercent, endColor.blue*fadePercent);
        fadeElapsedTime+=0.02;

        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
        }
        m_led.setData(m_ledBuffer);
      }).until(()->fadeElapsedTime>=time));
  }

  public Command fadeOut(Color endColor, double time){
    return runOnce(()->{
      fadeElapsedTime = 0;
    }).andThen(run(()->{
      double fadePercent = 1- (fadeElapsedTime / time);
      Color color=new Color(endColor.red*fadePercent, endColor.green*fadePercent, endColor.blue*fadePercent);
      fadeElapsedTime+=0.02;

      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
      }
      m_led.setData(m_ledBuffer);
    }).until(()->fadeElapsedTime>=time));
}

  int m_rainbowFirstPixelHue = 0;
  final double rainbowFadeInTime = 1;
  public Command rainbow() {
    return runOnce(()->{
      fadeElapsedTime = 0;
    }).andThen(run(() -> {
      double fadeFactor = fadeElapsedTime / rainbowFadeInTime;
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (m_rainbowFirstPixelHue + (i * 180 * 7 / m_ledBuffer.getLength())) % 180;
        // Set the value
        m_ledBuffer.setHSV(i, hue, 255, (int) (255 * fadeFactor));
      }
      // Increase by to make the rainbow "move"
      m_rainbowFirstPixelHue += 8;
      // Check bounds
      m_rainbowFirstPixelHue %= 180;
      m_led.setData(m_ledBuffer);
      fadeElapsedTime += (fadeElapsedTime < rainbowFadeInTime) ? .02 : 0;
    }).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }

  double pingPongStart = 0;
  public Command pingPongRight(Color color, double speed, double lineLength) {
    return runOnce(()->{
      pingPongStart = 0;
    }).andThen(run(()->{
      for (int i = 0; i < m_ledBuffer.getLength(); i ++) {
        if (i >= pingPongStart && i <= pingPongStart + lineLength) {
          m_ledBuffer.setLED(i, color);
        } else {
          m_ledBuffer.setLED(i, Color.kBlack);
        }
      }
      m_led.setData(m_ledBuffer);

      pingPongStart += speed;
    }).until(()->pingPongStart >= m_ledBuffer.getLength()));
  }

  public Command pingPongLeft(Color color, double speed, double lineLength) {
    return runOnce(()->{
      pingPongStart = m_ledBuffer.getLength();
    }).andThen(run(()->{
      for (int i = 0; i < m_ledBuffer.getLength(); i ++) {
        if (i >= pingPongStart && i <= pingPongStart + lineLength) {
          m_ledBuffer.setLED(i, color);
        } else {
          m_ledBuffer.setLED(i, Color.kBlack);
        }
      }
      m_led.setData(m_ledBuffer);

      pingPongStart -= speed;
    }).until(()->pingPongStart <= 0));
  }

  public Command doublePingPong(Color color1, Color color2, double speed, double lineLength) {
    return runOnce(()->{
      pingPongStart = 0;
    }).andThen(run(()->{
      double secondStart = m_ledBuffer.getLength() - pingPongStart;
      for (int i = 0; i < m_ledBuffer.getLength(); i ++) {
        if (i >= pingPongStart && i <= pingPongStart + lineLength) {
          m_ledBuffer.setLED(i, color1);
        } else {
          m_ledBuffer.setLED(i, Color.kBlack);
        }

        if(i >= secondStart && i <= secondStart + lineLength) {
          m_ledBuffer.setLED(i, color2);
        } 

      }

      m_led.setData(m_ledBuffer);

      pingPongStart += speed;
    }).until(()->pingPongStart >= m_ledBuffer.getLength()));
  }

  public Command enabledIdle() {
    return pingPongRight(Color.kPurple, 4, 12).repeatedly().withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command disabledIdle() {
    return doublePingPong(Color.kGreen, Color.kGreen, 4, 12).repeatedly().withTimeout(2).andThen(pingPongRight(Color.kPurple, 4, 12)).andThen(pingPongRight(Color.kGreenYellow, 4, 12)).andThen(pingPongLeft(Color.kAliceBlue, 3, 12)).andThen(
      fadeIn(Color.kGreen, .1).andThen(fadeOut(Color.kGreen, .4)).repeatedly().withTimeout(3)
    ).andThen(rainbow().withTimeout(2)).repeatedly()
    .ignoringDisable(true).withInterruptBehavior(InterruptionBehavior.kCancelSelf).onlyWhile(DriverStation::isDisabled);
  }       

}
