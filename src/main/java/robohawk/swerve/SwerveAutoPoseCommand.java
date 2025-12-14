package robohawk.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import robohawk.util.MotionProfile.State;
import robohawk.util.MotionProfile;
import robohawk.util.Utils;

import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * This command drives the robot to a target pose on the field. It handles
 * two different aspects of movement:
 * <ul>
 *
 *     <li>Rotation - if the new pose has a different heading than the
 *     starting pose when the command is run, we use a {@link MotionProfile}
 *     to smoothly rotate to the target heading.</li>
 *
 *     <li>Translation - if the new pose has a different position on the field
 *     than the starting pose when the command is run, we use a second
 *     {@link MotionProfile} to smoothly drive along a straight line to the
 *     target position.</li>
 *
 * </ul>
 *
 * <p>These are independent actions and are calculated separately, then combined
 * during execution. Neither is required - you can rotate without translating
 * (e.g. to align to a target heading), or translate without rotating (e.g. to
 * "scoot" to a fixed tx).</p>
 *
 * <p>This command is implemented so it doesn't depend on a specific swerve
 * drive implementation.</p>
 */
public class SwerveAutoPoseCommand extends Command {

    final SwerveAutoPoseConfig config;
    final Supplier<Pose2d> currentPoseGetter;
    final Consumer<ChassisSpeeds> driveFunction;
    final Function<Pose2d,Pose2d> targetPoseCalculator;
    final PIDController pidX;
    final PIDController pidY;
    final PIDController pidOmega;
    final Timer timer;
    boolean skipTranslation;
    boolean skipRotation;
    Pose2d startPose;
    Pose2d finalPose;
    MotionProfile rotationProfile;
    MotionProfile translationProfile;
    double cos;
    double sin;

    /**
     * Creates a {@link SwerveAutoPoseCommand}
     *
     * @param config configuration (required)
     * @param subsystem subsystem controlling the drive (required)
     * @param currentPoseGetter getter for the robot's current pose (required)
     * @param driveFunction method to accept robot-relative speeds (required)
     * @param targetPoseCalculator calculates the targer pose (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public SwerveAutoPoseCommand(SwerveAutoPoseConfig config,
                                 Subsystem subsystem,
                                 Supplier<Pose2d> currentPoseGetter,
                                 Consumer<ChassisSpeeds> driveFunction,
                                 Function<Pose2d,Pose2d> targetPoseCalculator) {

        Objects.requireNonNull(config);
        Objects.requireNonNull(subsystem);
        Objects.requireNonNull(currentPoseGetter);
        Objects.requireNonNull(driveFunction);
        Objects.requireNonNull(targetPoseCalculator);

        this.config = config;
        this.currentPoseGetter = currentPoseGetter;
        this.driveFunction = driveFunction;
        this.targetPoseCalculator = targetPoseCalculator;
        this.pidX = makePid(config.translateP, config.translateD);
        this.pidY = makePid(config.translateP, config.translateD);
        this.pidOmega = makePid(config.rotateP, config.rotateD);
        this.timer = new Timer();

        pidOmega.enableContinuousInput(-180.0, 180.0);

        addRequirements(subsystem);
    }

    private PIDController makePid(DoubleSupplier p, DoubleSupplier d) {
        return new PIDController(p.getAsDouble(), 0, d.getAsDouble());
    }

    // ===============================================================
    // INITIALIZATION
    // ===============================================================

    /**
     * Initializes the command by calculating the starting pose and the
     * motion profiles for translation and rotation
     */
    @Override
    public void initialize() {

        startPose = currentPoseGetter.get();
        finalPose = targetPoseCalculator.apply(startPose);
        Utils.log("[swerve-auto] moving from %s to %s",
                startPose,
                finalPose);

        initializeRotation(startPose);
        initializeTranslation(startPose);

        timer.restart();
    }

    // calculates the motion profile for rotation
    private void initializeRotation(Pose2d startPose) {

        // if we're not rotating, there's nothing to do
        skipRotation = startPose.getRotation().equals(finalPose.getRotation());
        if (skipRotation) {
            return;
        }

        // we calculate an "offset" around the circle based on its delta
        // from the starting position - starting at 0 degrees and ending
        // at the final heading. the sign of this tx may be positive or
        // negative depending on whether we're going left or right.
        double degrees = finalPose.getRotation()
                        .minus(startPose.getRotation())
                        .getDegrees();

        // we calculate a smooth profile from 0 degrees offset to the
        // target offset
        rotationProfile = new MotionProfile(
                config.rotateMaxVelocity.getAsDouble(),
                config.rotateMaxAcceleration.getAsDouble(),
                config.rotateRampTime.getAsDouble(),
                0.0,
                0.0,
                degrees);

        // reset PID
        Utils.resetPid(pidOmega,
                config.rotateP,
                config.rotateD,
                config.rotateTolerance);

    }

    // calculates the motion profile for translation
    private void initializeTranslation(Pose2d startPose) {

        // if we're not translating, there's nothing to do
        skipTranslation = startPose.getTranslation()
                .equals(finalPose.getTranslation());
        if (skipTranslation) {
            return;
        }

        // this is how far (in feet) we are moving from start to final
        // (this will always be a positive number)
        double distance = Utils.feetBetween(startPose, finalPose);

        // this is the angle of line between the start and final poses; cos
        // and sin will help us decompose straight-line movement along that
        // line into separate X and Y components
        Rotation2d angle = finalPose.getTranslation()
                .minus(startPose.getTranslation())
                .getAngle();
        cos = angle.getCos();
        sin = angle.getSin();

        // we will plan motion along the straight line between start and final
        translationProfile = new MotionProfile(
                config.translateMaxVelocity.getAsDouble(),
                config.translateMaxAcceleration.getAsDouble(),
                config.translateRampTime.getAsDouble(),
                0.0,
                0.0,
                distance);

        // reset PIDs
        Utils.resetPid(pidX,
                config.translateP,
                config.translateD,
                config.translateTolerance);
        Utils.resetPid(pidY,
                config.translateP,
                config.translateD,
                config.translateTolerance);

    }

    // ===============================================================
    // EXECUTION
    // ===============================================================

    @Override
    public void execute() {

        Pose2d currentPose = currentPoseGetter.get();

        // calculate the desired rotation and translation
        DesiredRotation desiredRotation = calculateRotation(
                currentPose,
                timer.get());
        DesiredTranslation desiredTranslation = calculateTranslation(
                currentPose,
                timer.get());

        // all these calculations assume field-relative movement, so when we
        // calculate speed we have to convert it
        ChassisSpeeds speeds = new ChassisSpeeds(
                desiredTranslation.desiredSpeed.getX(),
                desiredTranslation.desiredSpeed.getY(),
                desiredRotation.desiredSpeed.getRadians());
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                currentPose.getRotation());

        // let's also  calculate and display where we "should" be; this is
        // incredibly helpful for debugging
        Pose2d desiredPose = new Pose2d(
                desiredTranslation.desiredPosition,
                desiredRotation.desiredHeading);
        Utils.publishPose("AutoPoseNext", desiredPose);
        Utils.publishPose("AutoPoseFinal", finalPose);

        // make it so!
        driveFunction.accept(speeds);
    }

    private DesiredRotation calculateRotation(Pose2d currentPose, double time) {

        // if we're not rotating, there's nothing to do
        if (skipRotation) {
            return new DesiredRotation(
                    Rotation2d.kZero,
                    startPose.getRotation());
        }

        // this tells us both where we are supposed to be facing right now
        // (in degrees), as well as how fast we should be turning (in degrees
        // per second) and in what direction
        State desiredState = rotationProfile.sample(time);

        // the velocity is the base component of our turning speed - it's like
        // a "feedforward" component
        double desiredSpeed = desiredState.velocity();

        // this is the actual position where we should be at this time
        Rotation2d desiredHeading = startPose.getRotation()
                .plus(Rotation2d.fromDegrees(desiredState.position()));

        // we compare actual vs desired position to add feedback to speed and
        // correct for discrepancies
        desiredSpeed += Utils.applyClamp(
                pidOmega.calculate(currentPose.getRotation().getDegrees(), desiredHeading.getDegrees()),
                config.rotateMaxFeedback);

        return new DesiredRotation(
                Rotation2d.fromDegrees(desiredSpeed),
                desiredHeading);
    }

    private DesiredTranslation calculateTranslation(Pose2d currentPose, double time) {

        // if we're not translating, there's nothing to do
        if (skipTranslation) {
            return new DesiredTranslation(
                    Translation2d.kZero,
                    currentPose.getTranslation());
        }

        // this is tells us how far we should be along the straight line
        // between (in feet) between the start and final pose, and how fast we
        // should be going along that line (in feet per second)
        State desiredState = translationProfile.sample(time);

        // this decomposes the speed (in feet per second) into X and Y
        // components using the cos and sin we already calculated; as with
        // rotation this is "feedforward"
        double speedX = desiredState.velocity() * cos;
        double speedY = desiredState.velocity() * sin;

        // this does the same for position (but calculates it in meters, since
        // that's what Pose2d uses, and then updates desired speed based on
        // feedback
        double positionX = startPose.getX() + Units.feetToMeters(desiredState.position() * cos);
        double positionY = startPose.getY() + Units.feetToMeters(desiredState.position() * sin);

        // here's where we calculate feedback in the X and Y directions based
        // on how far off we are from the desired X/Y
         speedX += Utils.applyClamp(
                 pidX.calculate(currentPose.getX(), positionX),
                 config.translateMaxFeedback);
         speedY += Utils.applyClamp(
                 pidY.calculate(currentPose.getY(), positionY),
                 config.translateMaxFeedback);

        return new DesiredTranslation(
                new Translation2d(Units.feetToMeters(speedX), Units.feetToMeters(speedY)),
                new Translation2d(positionX, positionY));
    }

    // ===============================================================
    // FINISHING UP
    // ===============================================================

    @Override
    public boolean isFinished() {

        double time = timer.get();

        // rotation and translation are separate motions; one may complete
        // before the other, but we aren't done until both are complete
        boolean rotationDone = skipRotation || rotationProfile.isFinishedAt(time);
        boolean translationDone = skipTranslation || translationProfile.isFinishedAt(time);

        return rotationDone && translationDone;
    }

    @Override
    public void end(boolean interrupted) {

        Pose2d currentPose = currentPoseGetter.get();
        Rotation2d currentHeading = currentPose.getRotation();

        // since we determine completion based on the timing of the motion
        // profiles, we may not actually hit the target pose (e.g. if there
        // is an obstruction on the field, or our tuning is wrong). we will
        // log a warning about that - if this appears a lot, there may be a
        // problem with tuning

        double deltaDegrees = finalPose.getRotation()
                .minus(currentHeading)
                .getDegrees();
        if (Math.abs(deltaDegrees) > config.rotateTolerance.getAsDouble()) {
            Utils.log("[auto-pose] !!! FAILED ROTATE; delta is %.2f", deltaDegrees);
        } else {
            Utils.log("[auto-pose] done rotating");
        }

        double deltaFeet = Utils.feetBetween(finalPose, currentPose);
        if (Math.abs(deltaFeet) > config.translateTolerance.getAsDouble()) {
            Utils.log("[auto-pose] !!! FAILED TRANSLATE; delta is %.2f", deltaFeet);
        } else {
            Utils.log("[auto-pose] done translating");
        }

        SmartDashboard.putBoolean("SwerveAutoPoseCommand/Running?", false);
    }

    // ===============================================================
    // HELPERS
    // ===============================================================

    record DesiredRotation(Rotation2d desiredSpeed, Rotation2d desiredHeading) {

    }

    record DesiredTranslation(Translation2d desiredSpeed, Translation2d desiredPosition) {

    }
}
