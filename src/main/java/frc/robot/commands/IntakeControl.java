package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeControl extends CommandBase {
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final DoubleSupplier rightTrigger, leftTrigger;
    private final Trigger intake, outtake;

    public IntakeControl(DoubleSupplier rightTrigger, DoubleSupplier leftTrigger, Trigger intake, Trigger outtake) {
        addRequirements(intakeSubsystem);

        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;
        this.intake = intake;
        this.outtake = outtake;
    }

    @Override
    public void initialize() {
        intakeSubsystem.intakeOff();
    }

    @Override
    public void execute() {
        if(leftTrigger.getAsDouble() > 0.5) {
            intakeSubsystem.setIntakeMotor(leftTrigger.getAsDouble() * -0.4);
            intakeSubsystem.intakeExtend();
            armSubsystem.setArmPosition(Constants.ArmPosition.INTAKE);
        } else if(rightTrigger.getAsDouble() > 0.5) {
            intakeSubsystem.setIntakeMotor(rightTrigger.getAsDouble() * 0.4);
            intakeSubsystem.intakeExtend();
            armSubsystem.setArmPosition(Constants.ArmPosition.INTAKE);
        } else {
            if(intake.getAsBoolean())
                intakeSubsystem.setIntakeMotor(0.4);
            else if(outtake.getAsBoolean())
                intakeSubsystem.setIntakeMotor(-0.4);
            else
                intakeSubsystem.setIntakeMotor(0.0);
            intakeSubsystem.intakeRetract();
        }
    }
}
