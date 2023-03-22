package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

public class IntakeControl extends CommandBase {
    private final Intake intake = Intake.getInstance();
    private final DoubleSupplier rightTrigger, leftTrigger;

    public IntakeControl(DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {
        addRequirements(intake);

        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;
    }

    @Override
    public void execute() {
        if(rightTrigger.getAsDouble() > 0.5)
            intake.setIntakeMotor(rightTrigger.getAsDouble() * 0.4);
        else if(leftTrigger.getAsDouble() > 0.5)
            intake.setIntakeMotor(leftTrigger.getAsDouble() * -0.4);
        else
            intake.setIntakeMotor(0.0);
    }
}
