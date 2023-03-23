// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClawSubsystem;

public class ClawControl extends CommandBase {
    private final ClawSubsystem m_clawSubsystemSubsystem = ClawSubsystem.getInstance();
    private final Trigger m_lBumper, m_rBumper, m_x, m_y;

    public ClawControl(Trigger lBumper, Trigger rBumper, Trigger x, Trigger y) {
        addRequirements(m_clawSubsystemSubsystem);
        m_lBumper = lBumper;
        m_rBumper = rBumper;
        m_x = x;
        m_y = y;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_clawSubsystemSubsystem.clawOpenAction();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_clawSubsystemSubsystem.clawOpenCloseButton(m_rBumper.getAsBoolean(), m_lBumper.getAsBoolean());
        m_clawSubsystemSubsystem.clawButton(m_x.getAsBoolean(), m_y.getAsBoolean());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
