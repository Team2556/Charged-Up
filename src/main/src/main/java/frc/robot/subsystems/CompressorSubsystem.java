// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Ports;

public class CompressorSubsystem extends SubsystemBase {
    private final static CompressorSubsystem instance = getInstance();
    // Creates a new Compressor
    private final PneumaticHub m_ph = new PneumaticHub(Ports.pneumaticHubCANID);

    public void smartDashboardReset() {
        SmartDashboard.setDefaultBoolean("Enable Compressor", false);
        SmartDashboard.setDefaultBoolean("Disable Compressor", false);
    }

    public void update() {
        SmartDashboard.putBoolean("Digital Pressure Switch", m_ph.getPressureSwitch());

        // Get compressor running status and display on the dashboard
        SmartDashboard.putBoolean("Compressor Running", m_ph.getCompressor());

        // Enable Compressor through dashboard
        if (SmartDashboard.getBoolean("Enable Compressor", false)) {
            SmartDashboard.putBoolean("Enable Compressor", false);

            /*
            * Enable the compressor with digital sensor control.
            *
            * This will make the compressor run whenever the pressure switch is closed.
            * If open, (disconnected or reached max pressure), the compressor will shut
            * off.
            */
            m_ph.enableCompressorDigital();
        }

        // Disable Compressor through dashboard
        if (SmartDashboard.getBoolean("Disable Compressor", false)) {
            SmartDashboard.putBoolean("Disable Compressor", false);

            // Disable the compressor
            m_ph.disableCompressor();
        }
    }

    public static CompressorSubsystem getInstance() {
        if(instance == null)
            return new CompressorSubsystem();
        return instance;
    }
}