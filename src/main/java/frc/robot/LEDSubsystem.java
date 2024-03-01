// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDSubsystem extends SubsystemBase {

  AddressableLED m_led = new AddressableLED(0);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(240);

  /** Creates a new LEDController. */
  public LEDSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public Command setColor(Color color) {
    return this.runOnce(() -> {
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, (int)(color.red*255), (int)(color.green*255), (int)(color.blue*255));
      }
      m_led.setData(m_ledBuffer);
    });
  }

  public Command strobeLED(Color color, double period) {
    return setColor(color).andThen(
      new WaitCommand(period / 2)
    ).andThen(
      setColor(Color.kBlack)
    ).andThen(
      new WaitCommand(period/2)
    ).repeatedly();
  }

  /*public void periodic(){
    Color color=Color.kWhite;
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 255, 255);
    }
    m_led.setData(m_ledBuffer);
  }*/
}
