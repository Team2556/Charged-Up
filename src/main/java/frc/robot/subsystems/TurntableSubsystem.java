// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurntableSubsystem extends SubsystemBase {
  /** Creates a new Turntable. */
  static TalonSRX turntableMotor = new TalonSRX(12);
  public void TurntableRun() {
    turntableMotor.set(ControlMode.PercentOutput, -0.36);
  }

  DigitalInput leftIR = new DigitalInput(2);
  DigitalInput rightIR = new DigitalInput(1);
  boolean check = false;
  boolean check2 = false;

public void TurntableRevAction(boolean BButton) {
  if(BButton) {
    TurntableRev();
  }
}

/*public void TurntableOps() {
  if (!rightIR.get() && !leftIR.get()) {
    turntableMotor.set(ControlMode.PercentOutput, 0.0);
  } else {
    turntableMotor.set(ControlMode.PercentOutput, -0.4);
  }
  // SmartDashboard.putBoolean("leftIR", leftIR.get());
  // SmartDashboard.putBoolean("right", rightIR.get());
}*/
public boolean ToggleTurntable(Trigger button){
  if (button.getAsBoolean()){
  check = !check;
  }
  return check;
  
}
public void TurntableSpin(boolean flag) {
  if (flag) {
      if (!rightIR.get() && !leftIR.get()) {
        turntableMotor.set(ControlMode.PercentOutput, 0);
      }
      else {
        turntableMotor.set(ControlMode.PercentOutput, -0.36);
      }
    }
  else{
  turntableMotor.set(ControlMode.PercentOutput, 0);
  }
}

public void TurntableRev() {
    turntableMotor.set(ControlMode.PercentOutput, -0.36);
  }
 public void TurntableReset()
 {
    turntableMotor.set(ControlMode.PercentOutput, 0.0);
 }

}