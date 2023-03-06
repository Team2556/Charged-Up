package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class SwerveDrive extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveSubsystem m_swerveDrive;
    private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;
    private final Trigger m_gyroReset;
    private final boolean m_isFieldRelative;
    private static boolean hasReset = false;

    /**
     * Creates a new ExampleCommand.
     *
     * @param swerveDriveSubsystem The subsystem used by this command.
     */
    public SwerveDrive(SwerveSubsystem swerveDriveSubsystem, DoubleSupplier throttleInput, 
            DoubleSupplier strafeInput, DoubleSupplier rotationInput, Trigger gyroReset, boolean isFieldRelative) {
        m_swerveDrive = swerveDriveSubsystem;
        m_throttleInput = throttleInput;
        m_strafeInput = strafeInput;
        m_rotationInput = rotationInput;
        m_gyroReset = gyroReset;
        m_isFieldRelative = isFieldRelative;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDriveSubsystem);
    }

    public void gyroReset() {
        if (m_gyroReset.getAsBoolean()) {
            SwerveSubsystem.gyroZero();
        }
        // SmartDashboard.putBoolean("a", m_gyroReset.getAsBoolean());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(!hasReset) {
            m_swerveDrive.initializeAngle();
            hasReset = true;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double throttle = Math.abs(m_throttleInput.getAsDouble()) > 0.15 ? m_throttleInput.getAsDouble() : 0;
        double strafe = Math.abs(m_strafeInput.getAsDouble()) > 0.15 ? m_strafeInput.getAsDouble() : 0;
        double rotation = Math.abs(m_rotationInput.getAsDouble()) > 0.15 ? m_rotationInput.getAsDouble() : 0;
        // Forward/Back Throttle, Left/Right Strafe, Left/Right Turn
        m_swerveDrive.drive(throttle * 0.8, strafe * 0.8, rotation * 0.8, m_isFieldRelative, true);
        gyroReset();
    }
}
