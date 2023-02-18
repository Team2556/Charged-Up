package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class SwerveDrive extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveSubsystem m_swerveDrive;
    private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;
    private final boolean m_isFieldRelative;

    /**
     * Creates a new ExampleCommand.
     *
     * @param swerveDriveSubsystem The subsystem used by this command.
     */
    public SwerveDrive(SwerveSubsystem swerveDriveSubsystem, DoubleSupplier throttleInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, boolean isFieldRelative) {
        m_swerveDrive = swerveDriveSubsystem;
        m_throttleInput = throttleInput;
        m_strafeInput = strafeInput;
        m_rotationInput = rotationInput;
        m_isFieldRelative = isFieldRelative;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDriveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double throttle = Math.abs(m_throttleInput.getAsDouble()) > 0.05 ? m_throttleInput.getAsDouble() : 0;
        double strafe = Math.abs(m_strafeInput.getAsDouble()) > 0.05 ? m_strafeInput.getAsDouble() : 0;
        double rotation = Math.abs(m_rotationInput.getAsDouble()) > 0.05 ? m_rotationInput.getAsDouble() : 0;
        // Forward/Back Throttle, Left/Right Strafe, Left/Right Turn
        m_swerveDrive.drive(throttle * 0.2, strafe * 0.2, rotation * 0.2, m_isFieldRelative, true);
    }
}
