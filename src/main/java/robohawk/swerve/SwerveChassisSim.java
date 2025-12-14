package robohawk.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import robohawk.util.Utils;

import java.util.Arrays;
import java.util.Objects;

/**
 * Simulates a swerve chassis, to let you run swerve-related code in
 * simulation and test out commands.
 */
public class SwerveChassisSim {

    final SwerveDriveKinematics kinematics;
    final int moduleCount;

    // current chassis heading and module states. we use radians and meters,
    // because those are the units used by ChassisSpeeds, Pose2d etc.
    double currentHeading;
    double [] moduleVelocity;
    double [] moduleAngle;
    double [] moduleDistance;

    /**
     * Creates {@link SwerveChassisSim}
     *
     * @param kinematics the chassis kinematics (required)
     * @param initialPose the initial pose of the robot
     * @throws IllegalArgumentException if required parameters are null
     */
    public SwerveChassisSim(SwerveDriveKinematics kinematics,
                            Pose2d initialPose) {

        // we need kinematics; if there's no pose, we'll assume it's zero
        Objects.requireNonNull(kinematics);
        if (initialPose == null) {
            initialPose = Pose2d.kZero;
        }

        this.kinematics = kinematics;
        this.moduleCount = kinematics.getModules().length;
        this.currentHeading = initialPose.getRotation().getRadians();
        this.moduleVelocity = new double[moduleCount];
        this.moduleAngle = new double[moduleCount];
        this.moduleDistance = new double[moduleCount];
        Arrays.fill(moduleVelocity, 0.0);
        Arrays.fill(moduleAngle, 0.0);
        Arrays.fill(moduleDistance, 0.0);
    }

    /**
     * @return the current heading of the chassis (simulates a gyro)
     */
    public Rotation2d getCurrentHeading() {
        return Rotation2d.fromRadians(currentHeading);
    }

    /**
     * @return the current speed of the chassis
     */
    public ChassisSpeeds getCurrentSpeed() {
        SwerveModuleState [] states = new SwerveModuleState[4];
        for (int i=0; i<moduleCount; i++) {
            states[i] = new SwerveModuleState(
                moduleVelocity[i],
                Rotation2d.fromRadians(moduleAngle[i]));
        }
        return kinematics.toChassisSpeeds(states);
    }

    /**
     * @return current module positions
     */
    public SwerveModulePosition [] getModulePositions() {
        SwerveModulePosition [] positions = new SwerveModulePosition[4];
        for (int i=0; i<moduleCount; i++) {
            positions[i] = new SwerveModulePosition(moduleDistance[i], Rotation2d.fromRadians(moduleAngle[i]));
        }
        return positions;
    }

    /**
     * Sets module states. Use this to simulate driving. This should be
     * called every cycle, even if positions don't change, because it also
     * calculates how far we roll.
     *
     * @param states desired module states
     * @throws IllegalArgumentException if required parameters are null
     */
    public void setModuleStates(SwerveModuleState [] states) {

        Objects.requireNonNull(states);

        for (int i=0; i<moduleCount; i++) {

            // record current velocity and angle
            moduleVelocity[i] = states[i].speedMetersPerSecond;
            moduleAngle[i] = states[i].angle.getRadians();

            // calculate how far we've rolled in this unit time
            moduleDistance[i] += states[i].speedMetersPerSecond * Utils.DT;
        }

        // calculate how much the robot's heading has changed
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(states);
        currentHeading = MathUtil.angleModulus(
                currentHeading + speeds.omegaRadiansPerSecond * Utils.DT);
    }
}
