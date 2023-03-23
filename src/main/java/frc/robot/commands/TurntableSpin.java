// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurntableSubsystem;

public class TurntableSpin extends CommandBase {
    private final TurntableSubsystem m_turntableSubsystem = TurntableSubsystem.getInstance();

    public TurntableSpin(){
        addRequirements(m_turntableSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_turntableSubsystem.stopTurnTableMotor();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_turntableSubsystem.turntableSpin();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
