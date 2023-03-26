// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.Ports;

public class CompressorSubsystem extends SubsystemBase {
  /** Creates a new Compressor. */
  static PneumaticHub m_ph = new PneumaticHub(Swerve.Ports.pneumaticHubCANID); 

  public static void SmartDashboardReset() {
    SmartDashboard.putString("test", "test");
    SmartDashboard.setDefaultBoolean("Enable Compressor Digital", false);
    SmartDashboard.setDefaultBoolean("Disable Compressor", false);
  }

  public static void periodicUpdate() {
    SmartDashboard.putBoolean("Digital Pressure Switch",
    m_ph.getPressureSwitch());

    /**
    * Get compressor running status and display on Shuffleboard.
    */
    SmartDashboard.putBoolean("Compressor Running", m_ph.getCompressor());

    // Enable Compressor Digital button
    if (SmartDashboard.getBoolean("Enable Compressor Digital", false)) {
    SmartDashboard.putBoolean("Enable Compressor Digital", false);

    /**
    * Enable the compressor with digital sensor control.
    *
    * This will make the compressor run whenever the pressure switch is closed.
    * If open, (disconnected or reached max pressure), the compressor will shut
    * off.
    */
    m_ph.enableCompressorDigital();
    }

    // Disable Compressor button
    if (SmartDashboard.getBoolean("Disable Compressor", false)) {
    SmartDashboard.putBoolean("Disable Compressor", false);

    /**
    * Disable the compressor.
    */
    m_ph.disableCompressor();
    }
  }
  public CompressorSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // periodicUpdate();
  }
}
