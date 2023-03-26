// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.Ports;

public class Claw extends SubsystemBase {
  static DoubleSolenoid solenoid = 
    new DoubleSolenoid(9, PneumaticsModuleType.REVPH, 6, 7);

  static CANSparkMax clawSpin = new CANSparkMax(Swerve.Ports.clawNeo, MotorType.kBrushless);

  public static void clawOpenAction() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public static void clawCloseAction() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public static void clawReset() {
    solenoid.set(DoubleSolenoid.Value.kOff);
  }

  public static void clawOpenCloseButton(boolean rBumper, boolean lBumper) {
    SmartDashboard.putBoolean("RBumper", rBumper);
    SmartDashboard.putBoolean("LBumper", lBumper);
    if (rBumper && !lBumper) {
      clawOpenAction();
    } else if (!rBumper && lBumper) {
      clawCloseAction();
    } else {
      clawReset();
    }
  }


  public static void clawPullAction() {
    clawSpin.set(1.0);
  }

  public static void clawPushAction() {
    clawSpin.set(-1.0);
  }

  public static void clawStop() {
    clawSpin.set(0.0);
  }

  public static void clawButton(boolean xButton, boolean yButton) {
    if (xButton && !yButton) {
      clawPullAction();
    } else if (!xButton && yButton) {
      clawPushAction();
    } else {
      clawStop();
    }
  }

}