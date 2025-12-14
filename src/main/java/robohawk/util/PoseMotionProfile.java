package robohawk.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

/**
 * <p>Provides a "motion profile" for moving a robot in a straight line from
 * one pose to another. This includes two components: rotation and translation.
 * </p>
 *
 * <p>If the new pose has a different heading than the starting pose when the
 * command is run, we use a {@link MotionProfile} to smoothly rotate to the
 * target heading.
 * </p>
 *
 * <p>If the new pose has a different position on the field than the starting
 * pose when the command is run, we use a second {@link MotionProfile} to
 * smoothly drive along a straight line to the target position.
 * </p>
 *
 * These are independent actions and are calculated separately, then combined
 * during execution. Neither is required - you can rotate without translating
 * (e.g. to align to a target heading), or translate without rotating (e.g. to
 * "scoot" to a fixed tx).
 */
public class PoseMotionProfile {

    /**
     * Result of calculating a position along the profiled motion.
     *
     * @param speeds the speed of the robot
     * @param pose the pose of the robot
     */
    public record State(ChassisSpeeds speeds, Pose2d pose) {

    }

    MotionProfile translate;
    MotionProfile rotate;
    Pose2d startPose;
    double cos;
    double sin;

    /**
     * Creates an uninitialized {@link PoseMotionProfile}
     */
    public PoseMotionProfile() {
        translate = new MotionProfile();
        rotate = new MotionProfile();
    }

    // =============================================================
    // CONSTRAINTS
    // =============================================================

    /**
     * Resets the translation constraints for this profile, with a ramp time
     * of 0.
     *
     * @param maxVelocity maximum velocity in feet per second
     * @param maxAcceleration maximum acceleration in feet per second squared
     * @throws IllegalArgumentException if velocity is &lt;= 0
     * @throws IllegalArgumentException if acceleration is &lt;= 0
     */
    public void resetTranslationConstraints(double maxVelocity, double maxAcceleration) {
        resetTranslationConstraints(maxVelocity, maxAcceleration, 0.0);
    }

    /**
     * Resets the translation constraints for this profile.
     *
     * @param maxVelocity maximum velocity in feet per second
     * @param maxAcceleration maximum acceleration in feet per second squared
     * @param rampTime time in seconds to reach maximum acceleration
     * @throws IllegalArgumentException if velocity is &lt;= 0
     * @throws IllegalArgumentException if acceleration is &lt;= 0
     * @throws IllegalArgumentException if ramp time is &lt; 0
     */
    public void resetTranslationConstraints(double maxVelocity, double maxAcceleration, double rampTime) {
        if (maxVelocity <= 0) {
            throw new IllegalArgumentException("maxVelocity must be positive");
        }
        if (maxAcceleration <= 0) {
            throw new IllegalArgumentException("maxAcceleration must be positive");
        }
        if (rampTime < 0) {
            throw new IllegalArgumentException("rampTime must be non-negative");
        }
        translate.resetConstraints(
                maxVelocity,
                maxAcceleration,
                rampTime);
    }

    /**
     * Resets the rotation constraints for this profile, with a ramp time of 0.
     *
     * @param maxVelocity maximum velocity in degrees per second
     * @param maxAcceleration maximum acceleration in degrees per second squared
     * @throws IllegalArgumentException if velocity is &lt;= 0
     * @throws IllegalArgumentException if acceleration is &lt;= 0
     */
    public void resetRotationConstraints(double maxVelocity, double maxAcceleration) {
        resetRotationConstraints(maxVelocity, maxAcceleration, 0.0);
    }

    /**
     * Resets the rotation constraints for this profile.
     *
     * @param maxVelocity maximum velocity in degrees per second
     * @param maxAcceleration maximum acceleration in degrees per second squared
     * @param rampTime time in seconds to reach maximum acceleration
     * @throws IllegalArgumentException if velocity is &lt;= 0
     * @throws IllegalArgumentException if acceleration is &lt;= 0
     * @throws IllegalArgumentException if ramp time is &lt; 0
     */
    public void resetRotationConstraints(double maxVelocity, double maxAcceleration, double rampTime) {
        if (maxVelocity <= 0) {
            throw new IllegalArgumentException("maxVelocity must be positive");
        }
        if (maxAcceleration <= 0) {
            throw new IllegalArgumentException("maxAcceleration must be positive");
        }
        if (rampTime < 0) {
            throw new IllegalArgumentException("rampTime must be non-negative");
        }
        rotate.resetConstraints(maxVelocity, maxAcceleration, rampTime);
    }

    // =============================================================
    // MOTION PARAMETERS
    // =============================================================

    /**
     * Resets the motion bounds.
     *
     * @param startPose starting pose of the motion
     * @param finalPose final pose of the motion
     */
    public void resetMotion(Pose2d startPose, Pose2d finalPose) {

        double distance = Utils.feetBetween(startPose, finalPose);
        translate.resetMotion(0.0, 0.0, distance);

        double degrees = finalPose.getRotation().minus(startPose.getRotation()).getDegrees();
        rotate.resetMotion(0.0, 0.0, degrees);

        Rotation2d angle = finalPose.relativeTo(startPose).getTranslation().getAngle();
        cos = angle.getCos();
        sin = angle.getSin();

        this.startPose = startPose;
    }

    // =============================================================
    // SAMPLING
    // =============================================================

    /**
     * Samples the motion profile at the specified time.
     *
     * @param time the time in seconds since motion started
     * @return the calculated position and velocity at that time
     * @throws IllegalStateException if no constraints have been set
     * @throws IllegalStateException if no poses have been set
     */
    public State sample(double time) {

        if (startPose == null) {
            throw new IllegalStateException("startPose has not been set");
        }

        // we compute the translation profile using units of feet, but the
        // WPILib classes expect meters, so we will do the translation here
        MotionProfile.State stateT = translate.sample(time);
        double vm = Units.feetToMeters(stateT.velocity());
        double pm = Units.feetToMeters(stateT.position());

        MotionProfile.State stateR = rotate.sample(time);

        ChassisSpeeds speeds = new ChassisSpeeds(
                vm * cos,
                vm * sin,
                Math.toRadians(stateR.velocity()));

        Transform2d transform = new Transform2d(
                pm * cos,
                pm * sin,
                Rotation2d.fromDegrees(stateR.position()));

        return new State(speeds, startPose.transformBy(transform));
    }
}