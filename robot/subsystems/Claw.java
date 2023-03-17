// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve.Ports;

public class Claw extends SubsystemBase {
  static DoubleSolenoid solenoid = 
    new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);

  public static void clawOpenAction() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public static void clawCloseAction() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public static void clawReset() {
    solenoid.set(DoubleSolenoid.Value.kOff);
  }

  public static void clawOpenButton(boolean rBumper) {
    if (rBumper) {
      clawOpenAction();
    }
  }

  public static void clawCloseButton(boolean lBumper) {
    if (lBumper) {
      clawCloseAction();
    }
  }
}