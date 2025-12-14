package robohawk.util.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import robohawk.util.Utils;
import robohawk.util.vision.LimelightHelpers.PoseEstimate;

import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Supplies Limelight pose estimates based on the suggested filtering logic
 * for classic algorithm estimates (see the <a href="https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2">Limelight
 * docs</a> for more information)
 */
public class LimelightClassicSupplier implements Supplier<PoseEstimate> {

    /** Result of a pose estimate */
    public enum Status {

        /** No tag in view */
        NO_TAG,

        /** Tag is too ambiguous */
        TOO_AMBIGUOUS,

        /** Tag is too far away */
        TOO_FAR,

        /** Estimate is good */
        SUCCESS
    }

    final String limelightName;
    final DoubleSupplier maxAmbiguity;
    final DoubleSupplier maxDistance;
    Status status;

    /**
     * Creates a {@link LimelightClassicSupplier}
     *
     * @param limelightName name of the limelight (required)
     * @param maxAmbiguity supplier for maximum tag ambiguity (required)
     * @param maxDistance supplier for maximum tag distance (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public LimelightClassicSupplier(String limelightName,
                                    DoubleSupplier maxAmbiguity,
                                    DoubleSupplier maxDistance) {

        Objects.requireNonNull(limelightName);
        Objects.requireNonNull(maxAmbiguity);
        Objects.requireNonNull(maxDistance);

        this.limelightName = limelightName;
        this.maxAmbiguity = maxAmbiguity;
        this.maxDistance = maxDistance;
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

        // if there is no estimate, or it's not based on a single tag, we
        // will ignore it
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (estimate == null
                || estimate.tagCount != 1
                || estimate.rawFiducials.length != 1) {
            status = Status.NO_TAG;
            return null;
        }

        // we'll also ignore the estimate if it's too ambiguous or too far
        // away from the robot; if neither of those is true, we have a good
        // estimate and we'll submit it to the estimate consumer.
        double ambiguity = estimate.rawFiducials[0].ambiguity;
        double distance = estimate.rawFiducials[0].distToCamera;
        if (ambiguity > maxAmbiguity.getAsDouble()) {
            status = Status.TOO_AMBIGUOUS;
            return null;
        } else if (distance > maxDistance.getAsDouble()) {
            status = Status.TOO_FAR;
            return null;
        }

        // success!
        return estimate;
    }
}
