package robohawk.util.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import robohawk.util.Utils;
import robohawk.util.vision.LimelightHelpers.PoseEstimate;

import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Supplies Limelight pose estimates based on the suggested filtering logic
 * for MegaTag2 estimates (see the <a href="https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2">Limelight
 * docs</a> for more information)
 */
public class LimelightMegaTagSupplier implements Supplier<PoseEstimate> {

    /** Result of a pose estimate */
    public enum Status {

        /** No tag in view */
        NO_TAG,

        /** Robot is spinning too fast to trust estimates */
        SPINNING,

        /** Tag is too far away */
        TOO_FAR,

        /** Estimate is good */
        SUCCESS
    }

    final String limelightName;
    final Supplier<Rotation2d> headingGetter;
    final DoubleSupplier minArea;
    final DoubleSupplier maxYawRate;
    double previousYaw;
    Status status;

    /**
     * Creates a {@link LimelightMegaTagSupplier}
     *
     * @param limelightName name of the limelight (required)
     * @param headingGetter supplier for robot's current heading
     * @param minArea supplier for minimum tag area (required)
     * @param maxYawRate supplier for maximum yaw rate (required)
     * @throws IllegalArgumentException if required parameters are null
     */

    public LimelightMegaTagSupplier(String limelightName,
                                    Supplier<Rotation2d> headingGetter,
                                    DoubleSupplier minArea,
                                    DoubleSupplier maxYawRate) {

        Objects.requireNonNull(limelightName);
        Objects.requireNonNull(headingGetter);
        Objects.requireNonNull(minArea);
        Objects.requireNonNull(maxYawRate);

        this.limelightName = limelightName;
        this.headingGetter = headingGetter;
        this.minArea = minArea;
        this.maxYawRate = maxYawRate;
        this.previousYaw = headingGetter.get().getDegrees();
    }

    /**
     * @return the status of the last estimate provided (this can be useful
     * for debugging)
     */
    public Status getStatus() {
        return status;
    }

    /**
     * @return the latest valid pose estimate if there is one; null otherwise
     */
    @Override
    public PoseEstimate get() {

        // calculate the current heading and the yaw rate
        Rotation2d currentHeading = headingGetter.get();
        double currentYaw = currentHeading.getDegrees();
        double currentYawRate = (currentYaw - previousYaw) / Utils.DT;
        previousYaw = currentYaw;

        // MegaTag2 wants to know our heading for its calculations; we will
        // we will assume the robot is not pitching or yawing (you may want
        // to change this if you have a top-heavy robot)
        LimelightHelpers.SetRobotOrientation(limelightName, currentYaw, currentYawRate, 0.0, 0.0, 0.0, 0.0);

        // if we're spinning around too fast, LL estimates get wacky, so we
        // will ignore them
        if (currentYawRate > maxYawRate.getAsDouble()) {
            status = Status.SPINNING;
            return null;
        }

        // get an estimate; if there isn't one, or we don't have exactly
        // one item in view, or it's not a recognized AprilTag, we'll
        // ignore it
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (estimate == null || estimate.tagCount != 1 || estimate.rawFiducials.length != 1) {
            status = Status.NO_TAG;
            return null;
        }

        // if the id is too small (meaning too far away) we'll ignore it
        double area = estimate.avgTagArea;
        if (area < minArea.getAsDouble()) {
            status = Status.TOO_FAR;
            return null;
        }

        // otherwise, we're successful!
        return estimate;
    }
}
