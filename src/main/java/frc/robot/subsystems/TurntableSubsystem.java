// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Turntable Subsystem
 *  - Turntable Talon Motor
 *  - 2 Infrared Sensors to detect cone orientation
 *  - Manual Toggle to stop the turntable
 */
public class TurntableSubsystem extends SubsystemBase {
    private final static TurntableSubsystem instance = TurntableSubsystem.getInstance();
    private final TalonSRX turntableMotor = new TalonSRX(12);
    private final DigitalInput leftIR = new DigitalInput(2);
    private final DigitalInput rightIR = new DigitalInput(1);

    private boolean manualToggle = false;

    public TurntableSubsystem() {
        turntableMotor.configFactoryDefault();
        turntableMotor.setNeutralMode(NeutralMode.Brake);
        turntableMotor.setInverted(true);
    }

    public void turntableSpin() {
        if((rightIR.get() || leftIR.get()) && manualToggle)
            turntableMotor.set(ControlMode.PercentOutput, 0.36);
        else
            turntableMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setTurntableMotor(double speed) {
        turntableMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopTurnTableMotor() {
        setTurntableMotor(0.0);
    }

    public void setManualToggle(boolean manualToggle) {
        this.manualToggle = manualToggle;
    }

    public boolean getManualToggle() {
        return manualToggle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Right IR", rightIR.get());
        SmartDashboard.putBoolean("Left IR", leftIR.get());
    }

    public static TurntableSubsystem getInstance() {
        if(instance == null)
            return new TurntableSubsystem();
        return instance;
    }
}