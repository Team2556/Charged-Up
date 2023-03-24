// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.TurntableSubsystem;

public class TurntableSpin extends CommandBase {
    private final TurntableSubsystem m_turntableSubsystem = TurntableSubsystem.getInstance();
    private final Trigger fullSpeed, reverse;

    public TurntableSpin(Trigger fullSpeed, Trigger reverse) {
        addRequirements(m_turntableSubsystem);

        this.fullSpeed = fullSpeed;
        this.reverse = reverse;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_turntableSubsystem.stopTurnTableMotor();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(fullSpeed.getAsBoolean())
            m_turntableSubsystem.setTurntableMotor(-1.0);
        else  if(reverse.getAsBoolean())
            m_turntableSubsystem.setTurntableMotor(-0.36);
        else
            m_turntableSubsystem.turntableSpin();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
