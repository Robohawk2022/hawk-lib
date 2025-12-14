package robohawk.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import robohawk.util.Utils;
import robohawk.util.swerve.SwerveTeleopSpeedSupplier;

import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * This implements teleop driving using the {@link SwerveTeleopSpeedSupplier}.
 * This command is implemented so it doesn't depend on a specific swerve
 * drive implementation.
 */
public class SwerveTeleopCommand extends Command {

    final Consumer<ChassisSpeeds> driveFunction;
    final Supplier<ChassisSpeeds> speedSupplier;

    /**
     * Creates a {@link SwerveTeleopCommand}
     *
     * @param subsystem subsystem controlling the drive (required)
     * @param driveFunction method to accept robot-relative speeds (required)
     * @param speedSupplier speed supplier (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public SwerveTeleopCommand(Subsystem subsystem,
                               Consumer<ChassisSpeeds> driveFunction,
                               Supplier<ChassisSpeeds> speedSupplier) {

        Objects.requireNonNull(subsystem);
        Objects.requireNonNull(driveFunction);
        Objects.requireNonNull(speedSupplier);

        this.driveFunction = driveFunction;
        this.speedSupplier = speedSupplier;

        addRequirements(subsystem);
    }

    /** Logs the beginning of swerve teleop */
    @Override
    public void initialize() {
        Utils.log("[swerve-teleop] entering teleop");
    }

    /** Executes swerve teleop */
    @Override
    public void execute() {
        driveFunction.accept(speedSupplier.get());
    }
}
