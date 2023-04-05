// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.Object;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoGroup extends SequentialCommandGroup {
  /** Creates a new AutoGroup. */
  // private final ArmSubsystem m_armSubsystem;
  // private final CompressorSubsystem compressorSubsystem;
  // private final ClawSubsystem m_clawSubsystem;
  // private boolean retracted = false, grabbed = true;

 public AutoGroup() {
  // m_armSubsystem = ArmSubsystem.getInstance();
  // compressorSubsystem = CompressorSubsystem.getInstance();
  // m_clawSubsystem = ClawSubsystem.getInstance();
  // compressorSubsystem.update();

  // resetArm();
  // grabObject(); 

 
// Add your commands in the addCommands() call, e.g.
   // addCommands(new FooCommand(), new BarCommand());
   addCommands(
    new ResetArmCommand() ,
    // new GrabCommand() ,
    new ArmToOut() ,
    new DriveForward() /*,
    new ResetArmCommand(),
    new AutoBalance()*/);

    //  private boolean almostEqual(double a, double b, double eps){
    //   return Math.abs(a-b) < eps;
    // }

}
}