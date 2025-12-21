package robohawk.util.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import robohawk.util.swerve.SwervePoseCalculator.PoseType;
import robohawk.util.HawkUtils;

import java.util.Arrays;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * <p>This is a simple simulation of a swerve chassis. There is no physics
 * simulation; the "odometry" based pose will always reflect desired updates
 * exactly. This is meant to let you quickly test swerve-related code in
 * simulation to validate functionality.
 * </p>
 *
 * <p>You should make sure to call one of the <pre>drive</pre> methods each
 * scheduler loop, because that's where the pose calculations and
 * estimation updates happen.
 *
 * <p>You can contribute vision-based pose estimates using 
 * {@link #addVisionEstimate(Pose2d, double, Matrix)} and get both
 * "odometry" and vision-based estimates using
 * {@link #getLatestPoseEstimate(PoseType)}</p>
 */
public class SwerveChassisSim {

    final SwerveDriveKinematics kinematics;
    final SwerveKinematicsCalculator kinematicsCalculator;
    final SwervePoseCalculator poseCalculator;
    final BooleanSupplier overrideFusedHeading;
    final int moduleCount;

    // current chassis heading and module states. we use radians and meters,
    // because those are the units used by ChassisSpeeds, Pose2d etc.
    double currentHeading;
    double [] moduleVelocity;
    double [] moduleAngle;
    double [] moduleDistance;

    /**
     * Creates {@link SwerveChassisSim}
     * @param kinematics the chassis kinematics (required)
     * @param maximumWheelSpeed supplied for maximum wheel speed
     * @param useCosineCompensation indicates whether to use cosine compensation
     * @param overrideFusedHeading indicates whether to override the fused heading
     *                             with the odometry-only heading (see
     *                             {@link SwervePoseCalculator#updateLatestPoseEstimates(boolean)
     *                             for more information}
     * @param initialPose the initial pose of the robot
     * @throws IllegalArgumentException if required parameters are null
     */
    public SwerveChassisSim(SwerveDriveKinematics kinematics,
                            DoubleSupplier maximumWheelSpeed,
                            BooleanSupplier useCosineCompensation,
                            BooleanSupplier overrideFusedHeading,
                            Pose2d initialPose) {

        // we need kinematics; if there's no pose, we'll assume it's zero
        Objects.requireNonNull(kinematics);
        if (initialPose == null) {
            initialPose = Pose2d.kZero;
        }
        if (maximumWheelSpeed == null) {
            maximumWheelSpeed = () -> Double.POSITIVE_INFINITY;
        }
        if (useCosineCompensation == null) {
            useCosineCompensation = () -> false;
        }
        if (overrideFusedHeading == null) {
            overrideFusedHeading = () -> true;
        }

        this.kinematics = kinematics;
        this.moduleCount = kinematics.getModules().length;
        this.currentHeading = initialPose.getRotation().getRadians();
        this.moduleVelocity = new double[moduleCount];
        this.moduleAngle = new double[moduleCount];
        this.moduleDistance = new double[moduleCount];
        this.kinematicsCalculator = new SwerveKinematicsCalculator(
                kinematics,
                this::getModulePositions,
                maximumWheelSpeed,
                useCosineCompensation);
        this.poseCalculator = new SwervePoseCalculator(
                kinematics,
                this::getCurrentHeading,
                this::getModulePositions,
                initialPose);
        this.overrideFusedHeading = overrideFusedHeading;
        Arrays.fill(moduleVelocity, 0.0);
        Arrays.fill(moduleAngle, 0.0);
        Arrays.fill(moduleDistance, 0.0);
    }

    /**
     * @param type the desired type of pose estimate (if null, we will return
     *             the odometry-only estimate)
     * @return the latest pose estimate
     */
    public Pose2d getLatestPoseEstimate(PoseType type) {
        if (type == null) {
            type = PoseType.ODOMETRY;
        }
        return poseCalculator.getLatestPoseEstimate(type);
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
     * Adds a vision estimate to the pose calculator
     * @param pose the pose estimate
     * @param timestamp timestamp of pose estimate in seconds
     * @param stdDevs confidence in pose estimate
     */
    public void addVisionEstimate(Pose2d pose, double timestamp, Matrix<N3,N1> stdDevs) {
        if (pose != null) {
            poseCalculator.addVisionEstimate(pose, timestamp, stdDevs);
        }
    }

    /**
     * Drives the chassis using speeds
     * @param speeds robot-relative speeds (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public void drive(ChassisSpeeds speeds) {
        Objects.requireNonNull(kinematics);
        drive(kinematicsCalculator.calculateStates(speeds));
    }

    /**
     * Drives the chassis using module states
     * @param states desired module states (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public void drive(SwerveModuleState [] states) {

        Objects.requireNonNull(states);

        for (int i=0; i<moduleCount; i++) {

            // record current velocity and angle
            moduleVelocity[i] = states[i].speedMetersPerSecond;
            moduleAngle[i] = states[i].angle.getRadians();

            // calculate how far we've rolled in this unit time
            moduleDistance[i] += states[i].speedMetersPerSecond * HawkUtils.DT;
        }

        // calculate how much the robot's heading has changed
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(states);
        currentHeading = MathUtil.angleModulus(
                currentHeading + speeds.omegaRadiansPerSecond * HawkUtils.DT);

        // update pose estimate
        poseCalculator.updateLatestPoseEstimates(overrideFusedHeading.getAsBoolean());
    }
}
