package frc.robot;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// MONKEY!!!
// Adam is bad
// jacob is ok


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and motor controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 * @param <m_Motor>
 */
public class Intake<m_Motor> extends TimedRobot {
  private static final int kMotorPort = 0;
  private static final int kEncoderPortA = 0;
  private static final int kEncoderPortB = 1;
  private static final Object MotorType = null;
  private static final String CANSparkMaxLowLevel = null;
  private CANSparkMax m_Motor;
  private Encoder m_encoder;
  XboxController exampleXbox = new XboxController(0);
  private Object exampleCommandController;
  Trigger xButton = ((Trigger) exampleCommandController).RT(); 
  private int m_notifier;
  private static CANSparkMax intake = new CANSparkMax(CANSparkMaxLowLevel);

  @Override
  public void robotInit() {
    new PWMSparkMax(kMotorPort);
    new JoystickButton(exampleXbox, m_notifier);
    m_encoder = new Encoder(kEncoderPortA, kEncoderPortB);
    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.
    m_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    

  }{
  if (Trigger.getRawButtonPressed(0)) {
    turnIntakeOn(); // When pressed the intake turns on 
 }
 if (Trigger.getRawButtonReleased(0)) {
    turnIntakeOn(); // When released the intake turns off
 }
else;

 if (Trigger.getRawButton(0)) {
    turnIntakeOn();
 } else {
    turnIntakeOn();}
 }
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder", m_encoder.getDistance());
  }

  private void turnIntakeOn() {
  }

}
