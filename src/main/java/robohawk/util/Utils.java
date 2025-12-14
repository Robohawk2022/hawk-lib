package robohawk.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Basic utility functions that can be useful in different scenarios
 */
public class Utils {

    // ===========================================================
    // USEFUL CONSTANTS
    // ===========================================================

    /** Maximum volts for motors */
    public static final double MAX_VOLTS = 12.0;

    /** Time slice between calls to periodic() */
    public static final double DT = 0.02;

    /** Heading with no value */
    public static final Rotation2d NAN_ROTATION = new Rotation2d(Double.NaN);

    /** Pose with no values */
    public static final Pose2d NAN_POSE = new Pose2d(Double.NaN, Double.NaN, NAN_ROTATION);

    /** Speed with no values */
    public static final ChassisSpeeds NAN_SPEED = new ChassisSpeeds(Double.NaN, Double.NaN, Double.NaN);

    // ===========================================================
    // MISC STUFF
    // ===========================================================

    /**
     * @param volts a voltage value
     * @return the supplied value, clamped to +/- 12 volts
     */
    public static double clampVolts(double volts) {
        return MathUtil.clamp(volts, -MAX_VOLTS, MAX_VOLTS);
    }

    /**
     * @param value a value
     * @param limit a {@link DoubleSupplier} that will provide a clamp limit
     * @return the supplied value, clamped within limits defined by the supplier
     */
    public static double applyClamp(double value, DoubleSupplier limit) {
        double lim = Math.abs(limit.getAsDouble());
        return MathUtil.clamp(value, -lim, lim);
    }

    /**
     * @param value a value
     * @param lower a {@link DoubleSupplier} that will provide a lower limit
     * @param upper a {@link DoubleSupplier} that will provide an upper limit
     * @return the supplied value, clamped within limits defined by the supplier
     */
    public static double applyClamp(double value,
                                    DoubleSupplier lower,
                                    DoubleSupplier upper) {
        return MathUtil.clamp(value, lower.getAsDouble(), upper.getAsDouble());
    }

    /**
     * @param degrees an angle, specified in degrees
     * @return the supplied angle wrapped to (-180, 180)
     */
    public static double degreeModulus(double degrees) {
        return MathUtil.inputModulus(degrees, -180.0, 180.0);
    }

    /**
     * @param startPose starting pose
     * @param finalPose ending pose
     * @return the distance between the two poses in feet
     */
    public static double feetBetween(Pose2d startPose, Pose2d finalPose) {
        double meters = startPose.minus(finalPose).getTranslation().getNorm();
        return Units.metersToFeet(meters);
    }

    /**
     * Resets a PID controller to use the most recent tuning constants
     * and clear out accumulated error.
     *
     * @param pid the PID controller to reset
     * @param p a {@link DoubleSupplier} that will supply the new kP
     * @param d a {@link DoubleSupplier} that will supply the new kD
     * @param tolerance a {@link DoubleSupplier} that will supply the new tolerance
     */
    public static void resetPid(PIDController pid,
                                DoubleSupplier p,
                                DoubleSupplier d,
                                DoubleSupplier tolerance) {
        pid.setP(p.getAsDouble());
        pid.setD(d.getAsDouble());
        pid.setTolerance(tolerance.getAsDouble());
        pid.reset();
    }

    // ===========================================================
    // DRIVING & FIELD STUFF
    // ===========================================================

    /**
     * <p>Determines which {@link AprilTagFieldLayout} to use for AprilTag-
     * related methods. Default value is {@link AprilTagFields#kDefaultField}.
     * </p>
     *
     * <p>In 2025 there was an issue where the field measurements were
     * different depending on which vendor made the field; if something like
     * this happens again, you should change this before calling any of the
     * AprilTag related methods.
     * </p>
     */
    public static AprilTagFields FIELD = AprilTagFields.kDefaultField;

    // holds the field layout once we've loaded it (which happens the
    // first time we request id info)
    static AprilTagFieldLayout LAYOUT = null;

    /**
     * @return the default {@link AprilTagFieldLayout}
     */
    public static AprilTagFieldLayout getFieldLayout() {
        if (LAYOUT == null) {
            LAYOUT = AprilTagFieldLayout.loadField(FIELD);
        }
        return LAYOUT;
    }

    /**
     * @param id the ID of an AprilTag
     * @return information about the supplied AprilTag on the default field layout
     */
    public static Pose2d getAprilTagPose(int id) {
        Optional<Pose3d> pose = getFieldLayout().getTagPose(id);
        return pose.map(Pose3d::toPose2d).orElse(null);
    }

    static BooleanEntry isRedAlliance = null;

    /**
     * @return true if we are on the red alliance (if we're simulating we
     * fetch that from the dashboard; otherwise we get it from the driver
     * station)
     */
    public static boolean isRedAlliance() {
        if (RobotBase.isSimulation()) {
            if (isRedAlliance == null) {
                isRedAlliance = NetworkTableInstance.getDefault()
                        .getTable("FMSInfo")
                        .getBooleanTopic("IsRedAlliance")
                        .getEntry(false);
            }
            return isRedAlliance.get();
        } else {
            return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
        }
    }

    /** @return opposite of {@link #isRedAlliance()} */
    public static boolean isBlueAlliance() {
        return !isRedAlliance();
    }

    /**
     * @param speeds chassis speeds
     * @param tolerance a {@link DoubleSupplier} that will return a tolerance
     *                 in feet per second
     * @return true if the supplied speeds include an XY translation
     * greater than the supplied tolerance (in feet per second) in
     * either direction
     */
    public static boolean isTranslatingMoreThan(ChassisSpeeds speeds, DoubleSupplier tolerance) {
        double tol = tolerance.getAsDouble();
        return Math.abs(Units.metersToFeet(speeds.vxMetersPerSecond)) > tol
                || Math.abs(Units.metersToFeet(speeds.vyMetersPerSecond)) > tol;
    }

    /**
     * @param speeds chassis speeds
     * @return true if the supplied speeds include a rotation
     */
    public static boolean isRotating(ChassisSpeeds speeds) {
        return Math.abs(speeds.omegaRadiansPerSecond) > 0.0;
    }

    /** Prefix and loggers for logging pose structs */
    public static final String POSE_LOGGING_PREFIX = "SmartDashboard/SwerveDriveSubsystem/Structs/";

    static final Map<String,StructPublisher<Pose2d>> posePublishers = new HashMap<>();

    /**
     * Publish a pose to the dashboard (automatically adds the "SmartDashboard"
     * prefix so it will show up under that topic in the dashboard)
     *
     * @param key the key under which to publish the pose
     * @param val the pose to publish
     */
    public static void publishPose(String key, Pose2d val) {

        // see if a publisher already exists
        StructPublisher<Pose2d> publisher = posePublishers.get(key);

        // create it if it doesn't (we add the SmartDashboard prefix so
        // it shows up next to other values we publish)
        if (publisher == null) {
            publisher = NetworkTableInstance.getDefault()
                    .getStructTopic(POSE_LOGGING_PREFIX+key, Pose2d.struct)
                    .publish();
            posePublishers.put(key, publisher);
        }

        // if there's no pose specified, we'll publish on with NaN values
        if (val == null) {
            val = NAN_POSE;
        }

        publisher.set(val);
    }

    // ===========================================================
    // LOGGING & PREFERENCES
    // ===========================================================

    /**
     * Formats and writes a message to Rio log after formatting (adds a newline
     * to the end of the message if there isn't one)
     *
     * @param text the text of the log message
     * @param args arguments for the message
     */
    public static void log(String text, Object... args) {
        System.out.printf(text, args);
        if (!text.endsWith("%n")) {
            System.out.println();
        }
    }

    /**
     * @param name the name of a configuration property
     * @param defaultValue a default value for the property
     * @return a {@link BooleanSupplier} providing access to the specified configuration
     * preference (this also has the side effect of saving the value, if it's not already
     * persisted on the robot)
     */
    public static BooleanSupplier pref(String name, boolean defaultValue) {
        log("[util] registering pref %s = %s", name, defaultValue);
        Preferences.initBoolean(name, defaultValue);
        return () -> Preferences.getBoolean(name, defaultValue);
    }

    /**
     * @param name the name of a configuration property
     * @param defaultValue a default value for the property
     * @return a {@link DoubleSupplier} providing access to the specified configuration
     * preference (this also has the side effect of saving the value, if it's not already
     * persisted on the robot)
     */
    public static DoubleSupplier pref(String name, double defaultValue) {
        log("[util] registering pref %s = %.2f", name, defaultValue);
        Preferences.initDouble(name, defaultValue);
        return () -> Preferences.getDouble(name, defaultValue);
    }
}
