// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turntable extends SubsystemBase {
  /** Creates a new Turntable. */
  static TalonSRX turntableMotor = new TalonSRX(2);
  public static void TurntableDef() {
    turntableMotor.set(ControlMode.PercentOutput, 0.5);
  }

public static void TurntableRevAction(boolean BButton) {
  if(BButton) {
    TurntableRev();
  }
}
public static void TurntableRev() {
    turntableMotor.set(ControlMode.PercentOutput, -0.5);
  }
 public static void TurntableReset()
 {
    turntableMotor.set(ControlMode.PercentOutput, 0.0);
 }
}
