package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class SwerveDrive extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveSubsystem m_swerveDrive;
    private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;
    private final Trigger m_gyroReset;
    private static boolean precisionMode = false;
    private double regularScalar = 1.0, precisionScalar = 0.2;

    /**
     * Creates a new ExampleCommand.
     *
     * @param swerveDriveSubsystem The subsystem used by this command.
     */
    public SwerveDrive(SwerveSubsystem swerveDriveSubsystem, DoubleSupplier throttleInput, 
            DoubleSupplier strafeInput, DoubleSupplier rotationInput, Trigger gyroReset) {
        m_swerveDrive = swerveDriveSubsystem;
        m_throttleInput = throttleInput;
        m_strafeInput = strafeInput;
        m_rotationInput = rotationInput;
        m_gyroReset = gyroReset;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDriveSubsystem);
    }

    public void gyroReset() {
        if (m_gyroReset.getAsBoolean()) {
            m_swerveDrive.gyroZero();
        }
    }

    @Override
    public void initialize() {
        m_swerveDrive.setIsFieldRelative(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double throttle = Math.abs(m_throttleInput.getAsDouble()) > 0.15 ? m_throttleInput.getAsDouble() : 0;
        double strafe = Math.abs(m_strafeInput.getAsDouble()) > 0.15 ? m_strafeInput.getAsDouble() : 0;
        double rotation = Math.abs(m_rotationInput.getAsDouble()) > 0.15 ? m_rotationInput.getAsDouble() : 0;
        // Forward/Back Throttle, Left/Right Strafe, Left/Right Turn
        m_swerveDrive.drive(throttle * (getPrecisionMode() ? precisionScalar : regularScalar),
                strafe * (getPrecisionMode() ? precisionScalar : regularScalar),
                rotation * (getPrecisionMode() ? precisionScalar : regularScalar), true);
        gyroReset();
    }

    public static void setPrecisionMode(boolean precisionMode) {
        SwerveDrive.precisionMode = precisionMode;
    }

    public static boolean getPrecisionMode() {
        return precisionMode;
    }
}
