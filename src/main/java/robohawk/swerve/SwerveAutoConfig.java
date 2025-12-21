package robohawk.swerve;

import robohawk.util.HawkUtils;

import java.util.function.DoubleSupplier;

import static robohawk.util.HawkUtils.pref;

/**
 * Configuration properties for autonomous swerve positioning. Properties are
 * managed with {@link edu.wpi.first.wpilibj.Preferences}, which means the
 * following:
 * <ul>
 *
 *     <li>The current value is exposed via a {@link DoubleSupplier} so it
 *     will update immediately if the corresponding property is changed in
 *     the dashboard.</li>
 *
 *     <li>The values configured in the {@link SwerveTeleopConfig.Builder} are defaults; if
 *     someone has changed them on the RoboRIO the updated value will be
 *     used instead.</li>
 *
 * </ul>
 */
public class SwerveAutoConfig {

    /** Maximum velocity for rotation in degrees per second */
    public final DoubleSupplier rotateMaxVelocity;

    /** Maximum acceleration for rotation in degrees per second squared */
    public final DoubleSupplier rotateMaxAcceleration;

    /** P value for rotation */
    public final DoubleSupplier rotateP;

    /** D value for rotation */
    public final DoubleSupplier rotateD;

    /** Maximum feedback velocity for rotation in degrees per second */
    public final DoubleSupplier rotateMaxFeedback;

    /** Maximum position tolerance for rotation in degrees */
    public final DoubleSupplier rotateTolerance;

    /** Maximum velocity for translation in feet per second */
    public final DoubleSupplier translateMaxVelocity;

    /** Maximum acceleration for translation in feet per second squared */
    public final DoubleSupplier translateMaxAcceleration;

    /** P value for translation */
    public final DoubleSupplier translateP;

    /** D value for translation */
    public final DoubleSupplier translateD;

    /** Maximum feedback velocity for translation in feet per second */
    public final DoubleSupplier translateMaxFeedback;

    /** Maximum position tolerance for translation in inches */
    public final DoubleSupplier translateTolerance;

    private SwerveAutoConfig(DoubleSupplier rotateMaxVelocity,
                             DoubleSupplier rotateMaxAcceleration,
                             DoubleSupplier rotateP,
                             DoubleSupplier rotateD,
                             DoubleSupplier rotateMaxFeedback,
                             DoubleSupplier rotateTolerance,
                             DoubleSupplier translateMaxVelocity,
                             DoubleSupplier translateMaxAcceleration,
                             DoubleSupplier translateP,
                             DoubleSupplier translateD,
                             DoubleSupplier translateMaxFeedback,
                             DoubleSupplier translateTolerance) {
        this.rotateMaxVelocity = rotateMaxVelocity;
        this.rotateMaxAcceleration = rotateMaxAcceleration;
        this.rotateP = rotateP;
        this.rotateD = rotateD;
        this.rotateMaxFeedback = rotateMaxFeedback;
        this.rotateTolerance = rotateTolerance;
        this.translateMaxVelocity = translateMaxVelocity;
        this.translateMaxAcceleration = translateMaxAcceleration;
        this.translateP = translateP;
        this.translateD = translateD;
        this.translateMaxFeedback = translateMaxFeedback;
        this.translateTolerance = translateTolerance;
    }

    /**
     * @return a {@link Builder} for configuration
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Builder class for configuration. When you use this, the property
     * values will be managed via WPILib Preferences. Preferences will be
     * registered with the following names:
     * <ul>
     *     <li><pre>${prefix}/Rotate/MaxVelocity</pre> - default 720.0</li>
     *     <li><pre>${prefix}/Rotate/MaxAcceleration</pre> - default 1440.0</li>
     *     <li><pre>${prefix}/Rotate/kP</pre> - default 0.4</li>
     *     <li><pre>${prefix}/Rotate/kD</pre> - default 0.0</li>
     *     <li><pre>${prefix}/Rotate/MaxFeedback</pre> - default 360.0</li>
     *     <li><pre>${prefix}/Rotate/Tolerance</pre> - default 1.0</li>
     *     <li><pre>${prefix}/Translate/MaxVelocity</pre> - default 15.0</li>
     *     <li><pre>${prefix}/Translate/MaxAcceleration</pre> - default 30.0</li>
     *     <li><pre>${prefix}/Translate/kP</pre> - default 2.0</li>
     *     <li><pre>${prefix}/Translate/kD</pre> - default 0.1</li>
     *     <li><pre>${prefix}/Translate/MaxFeedback</pre> - default 10.0</li>
     *     <li><pre>${prefix}/Translate/Tolerance</pre> - default 2.0</li>
     * </ul>
     * See {@link HawkUtils#pref(String, double)} for some discussion of how
     * preferences work.
     */
    public static class Builder {

        String prefix = "SwerveAuto";
        double rotateMaxVelocity = 720.0;
        double rotateMaxAcceleration = 1440.0;
        double rotateP = 0.4;
        double rotateD = 0.0;
        double rotateMaxFeedback = 360.0;
        double rotateTolerance = 1.0;
        double translateMaxVelocity = 15.0;
        double translateMaxAcceleration = 30.0;
        double translateP = 2.0;
        double translateD = 0.1;
        double translateMaxFeedback = 10.0;
        double translateTolerance = 2.0;

        /**
         * @param prefix prefix for fetching properties from
         * {@link edu.wpi.first.wpilibj.Preferences}
         * @return this builder
         */
        public Builder prefix(String prefix) {
            this.prefix = prefix;
            return this;
        }

        /**
         * @param rotateMaxVelocity default value for max rotational velocity
         *                          (will be overridden by the corresponding
         *                          preference if it already exists)
         * @return this builder
         */
        public Builder rotateMaxVelocity(double rotateMaxVelocity) {
            this.rotateMaxVelocity = rotateMaxVelocity;
            return this;
        }

        /**
         * @param rotateMaxAcceleration default value for max rotational
         *                              acceleration (will be overridden by the
         *                              corresponding preference if it already
         *                              exists)
         * @return this builder
         */
        public Builder rotateMaxAcceleration(double rotateMaxAcceleration) {
            this.rotateMaxAcceleration = rotateMaxAcceleration;
            return this;
        }

        /**
         * @param rotateP default value for rotational P constant (will be
         *                overridden by the corresponding preference if it
         *                already exists)
         * @return this builder
         */
        public Builder rotateP(double rotateP) {
            this.rotateP = rotateP;
            return this;
        }

        /**
         * @param rotateD default value for rotational D constant (will be
         *                overridden by the corresponding preference if it
         *                already exists)
         * @return this builder
         */
        public Builder rotateD(double rotateD) {
            this.rotateD = rotateD;
            return this;
        }

        /**
         * @param rotateMaxFeedback default value for max rotational feedback
         *                          (will be overridden by the corresponding
         *                          preference if it already exists)
         * @return this builder
         */
        public Builder rotateMaxFeedback(double rotateMaxFeedback) {
            this.rotateMaxFeedback = rotateMaxFeedback;
            return this;
        }

        /**
         * @param rotateTolerance default value for rotational tolerance (will
         *                        be overridden by the corresponding preference
         *                        if it already exists)
         * @return this builder
         */
        public Builder rotateTolerance(double rotateTolerance) {
            this.rotateTolerance = rotateTolerance;
            return this;
        }

        /**
         * @param translateMaxVelocity default value for max translational
         *                             velocity (will be overridden by the
         *                             corresponding preference if it already
         *                             exists)
         * @return this builder
         */
        public Builder translateMaxVelocity(double translateMaxVelocity) {
            this.translateMaxVelocity = translateMaxVelocity;
            return this;
        }

        /**
         * @param translateMaxAcceleration default value for max translational
         *                                 acceleration (will be overridden by
         *                                 the corresponding preference if it
         *                                 already exists)
         * @return this builder
         */
        public Builder translateMaxAcceleration(double translateMaxAcceleration) {
            this.translateMaxAcceleration = translateMaxAcceleration;
            return this;
        }

        /**
         * @param translateP default value for translational P constant (will
         *                   be overridden by the corresponding preference if
         *                   it already exists)
         * @return this builder
         */
        public Builder translateP(double translateP) {
            this.translateP = translateP;
            return this;
        }

        /**
         * @param translateD default value for translational D constant (will
         *                   be overridden by the corresponding preference if
         *                   it already exists)
         * @return this builder
         */
        public Builder translateD(double translateD) {
            this.translateD = translateD;
            return this;
        }

        /**
         * @param translateMaxFeedback default value for max translational
         *                             feedback (will be overridden by the
         *                             corresponding preference if it already
         *                             exists)
         * @return this builder
         */
        public Builder translateMaxFeedback(double translateMaxFeedback) {
            this.translateMaxFeedback = translateMaxFeedback;
            return this;
        }

        /**
         * @param translateTolerance default value for translational tolerance
         *                           (will be overridden by the corresponding
         *                           preference if it already exists)
         * @return this builder
         */
        public Builder translateTolerance(double translateTolerance) {
            this.translateTolerance = translateTolerance;
            return this;
        }

        /**
         * @return a new {@link SwerveAutoConfig}
         */
        public SwerveAutoConfig build() {
            return new SwerveAutoConfig(
                    pref(prefix+"/Rotate/MaxVelocity", rotateMaxVelocity),
                    pref(prefix+"/Rotate/MaxAcceleration", rotateMaxAcceleration),
                    pref(prefix+"/Rotate/kP", rotateP),
                    pref(prefix+"/Rotate/kD", rotateD),
                    pref(prefix+"/Rotate/MaxFeedback", rotateMaxFeedback),
                    pref(prefix+"/Rotate/Tolerance", rotateTolerance),
                    pref(prefix+"/Translate/MaxVelocity", translateMaxVelocity),
                    pref(prefix+"/Translate/MaxAcceleration", translateMaxAcceleration),
                    pref(prefix+"/Translate/kP", translateP),
                    pref(prefix+"/Translate/kD", translateD),
                    pref(prefix+"/Translate/MaxFeedback", translateMaxFeedback),
                    pref(prefix+"/Translate/Tolerance", translateTolerance));
        }
    }
}
