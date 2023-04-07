// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoGroup extends SequentialCommandGroup {
  // DoubleSupplier sup = () -> .5;
  // DoubleSupplier sup1 = () -> 0;
  // DoubleSupplier sup2 = () -> 0;
  // BooleanSupplier sup3 = () -> true;
  // Trigger T = new Trigger( sup3);
  // private final ArmSubsystem m_armSubsystem;
  // private final CompressorSubsystem compressorSubsystem;
  // private final ClawSubsystem m_clawSubsystem;
  // private boolean retracted = false, grabbed = true;

 public AutoGroup() {
 
// Add your commands in the addCommands() call, e.g.
   // addCommands(new FooCommand(), new BarCommand());
   addCommands(
    // new ResetArmCommand(),
    // new ArmToOut(),
    // new DriveForwardByOdometry(),
    // new DropAndRetract(),
    // new DriveBackByOdometry(0.0, false),    
    // new DriveSideways(),
    // new DriveBackByOdometry(2.0, false));

    // new ResetArmCommand(),
    // new ArmToOut(),
    // new DriveForwardByOdometry(),
    // new DropAndRetract(),
    // new DriveBackByOdometry(2.25, true)); 
    new AutoBalance());  

    //new AutoBalance();
}
}