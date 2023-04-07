// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Ports.*;

public class ClawSubsystem extends SubsystemBase {
    private final static ClawSubsystem instance = getInstance();
    public final DoubleSolenoid grabSolenoid = new DoubleSolenoid(
          pneumaticHubCANID, PneumaticsModuleType.REVPH, grabSolenoidForward, grabSolenoidReverse);
    private final CANSparkMax clawSpin = new CANSparkMax(clawNeoPort, MotorType.kBrushless);
    private final Timer timer = new Timer();

    public ClawSubsystem() {
        timer.start();
    }

    public void clawOpenAction() {
        grabSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void clawCloseAction() {
        grabSolenoid.set(DoubleSolenoid.Value.kReverse);   
    }

    public void clawReset() {
        grabSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    public String clawCheck() {
        return grabSolenoid.get().toString();
    }

    public void clawOpenCloseButton(boolean rBumper, boolean lBumper) {
        if (rBumper && !lBumper) {
            clawOpenAction();
        } else if (!rBumper && lBumper) {
            clawCloseAction();
        } else {
            clawReset();
        }
    }


    public void resetTimer() {
        timer.reset();
    }

    public void runClawForSeconds(double seconds, double speed) {
        if(timer.get() > seconds)
            clawSpin.set(0.0);
        else
            clawSpin.set(speed);
    }

    public void clawPullAction() {
        clawSpin.set(0.3);
    }

    public void clawPushAction() {
       clawSpin.set(-0.3);
    }

    public void clawStop() {
        clawSpin.set(0.0);
    }

    public void clawButton(boolean xButton, boolean yButton) {
       if (xButton && !yButton) {
            clawPullAction();
        } else if (!xButton && yButton) {
            clawPushAction();
        } else {
            clawStop();
        }
    }

    public static ClawSubsystem getInstance() {
        if(instance == null)
            return new ClawSubsystem();
        return instance;
    }
}