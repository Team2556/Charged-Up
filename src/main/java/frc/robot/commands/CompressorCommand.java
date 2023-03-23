// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CompressorSubsystem;

public class CompressorCommand extends CommandBase {
    private final CompressorSubsystem m_compressorSystem = CompressorSubsystem.getInstance();

    public CompressorCommand() {
        addRequirements(m_compressorSystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_compressorSystem.smartDashboardReset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_compressorSystem.update();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
