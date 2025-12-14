package robohawk.swerve;

import robohawk.util.Utils;

import java.util.function.DoubleSupplier;

/**
 * Managed configuration properties for swerve auto/targeting commands.
 * Properties are managed with {@link edu.wpi.first.wpilibj.Preferences},
 * which means the following:
 * <ul>
 *
 *     <li>The current value is exposed via a {@link DoubleSupplier} so it
 *     will update immediately if the corresponding property is changed in
 *     the dashboard.</li>
 *
 *     <li>The values configured in the {@link Builder} are defaults; if
 *     someone has changed them on the RoboRIO the updated value will be
 *     used instead.</li>
 *
 * </ul>
 *
 * Property names are as follows:
 * <ul>
 *     <li><pre>${prefix}/Rotate/MaxDegreesPerSec</pre></li>
 *     <li><pre>${prefix}/Rotate/MaxAcceleration</pre></li>
 *     <li><pre>${prefix}/Rotate/RampTime</pre></li>
 *     <li><pre>${prefix}/Rotate/kP</pre></li>
 *     <li><pre>${prefix}/Rotate/kD</pre></li>
 *     <li><pre>${prefix}/Rotate/MaxFeedback</pre></li>
 *     <li><pre>${prefix}/Rotate/Tolerance</pre></li>
 *     <li><pre>${prefix}/Translate/MaxFeetPerSec</pre></li>
 *     <li><pre>${prefix}/Translate/MaxAcceleration</pre></li>
 *     <li><pre>${prefix}/Translate/RampTime</pre></li>
 *     <li><pre>${prefix}/Translate/kP</pre></li>
 *     <li><pre>${prefix}/Translate/kD</pre></li>
 *     <li><pre>${prefix}/Translate/MaxFeedback</pre></li>
 *     <li><pre>${prefix}/Translate/Tolerance</pre></li>
 * </ul>
 */
public class SwerveAutoPoseConfig {

    // ==================================================================
    // ROTATING
    // ==================================================================

    /**
     * Maximum velocity for autorotation in degrees per second
     * (default is 720.0)
     */
    public final DoubleSupplier rotateMaxVelocity;

    /**
     * Maximum acceleration for autorotation in degrees per second squared
     * (default is 1440.0)
     */
    public final DoubleSupplier rotateMaxAcceleration;

    /**
     * Ramp time in seconds for rotation (default is 0.2)
     */
    public final DoubleSupplier rotateRampTime;

    /**
     * Proportional feedback constant for angle correction during auto
     * rotation (default is 0.4)
     */
    public final DoubleSupplier rotateP;

    /**
     * Derivative feedback constant for angle correction during auto rotation
     * (default is 0.0)
     */
    public final DoubleSupplier rotateD;

    /**
     * Maximum feedback value for rotation in degrees per second
     * (default is 180.0)
     */
    public final DoubleSupplier rotateMaxFeedback;

    /**
     * How close in degrees to the target angle will we consider successful?
     * (default is 1.0)
     */
    public final DoubleSupplier rotateTolerance;

    // ==================================================================
    // TRANSLATING
    // ==================================================================

    /**
     * Maximum velocity for auto translation in feet per second
     * (default is 15.0)
     */
    public final DoubleSupplier translateMaxVelocity;

    /**
     * Maximum acceleration for auto translation in feet per second squared
     * (default is 30.0)
     */
    public final DoubleSupplier translateMaxAcceleration;

    /**
     * Ramp time in seconds for translation (default is 0.2)
     */
    public final DoubleSupplier translateRampTime;

    /**
     * Proportional feedback constant for position correction during auto
     * translation (default is 2.0)
     */
    public final DoubleSupplier translateP;

    /**
     * Derivative feedback constant for position correction during auto
     * translation (default is 0.0)
     */
    public final DoubleSupplier translateD;

    /**
     * Maximum feedback value for translation in feet per second
     * (default is 10.0)
     */
    public final DoubleSupplier translateMaxFeedback;

    /**
     * How close in inches to the target position will we consider successful?
     * (default is 2.0)
     */
    public final DoubleSupplier translateTolerance;

    private SwerveAutoPoseConfig(DoubleSupplier rotateMaxVelocity,
                                 DoubleSupplier rotateMaxAcceleration,
                                 DoubleSupplier rotateRampTime,
                                 DoubleSupplier rotateP,
                                 DoubleSupplier rotateD,
                                 DoubleSupplier rotateMaxFeedback,
                                 DoubleSupplier rotateTolerance,
                                 DoubleSupplier translateMaxVelocity,
                                 DoubleSupplier translateMaxAcceleration,
                                 DoubleSupplier translateRampTime,
                                 DoubleSupplier translateP,
                                 DoubleSupplier translateD,
                                 DoubleSupplier translateMaxFeedback,
                                 DoubleSupplier translateTolerance) {
        this.rotateMaxVelocity = rotateMaxVelocity;
        this.rotateMaxAcceleration = rotateMaxAcceleration;
        this.rotateRampTime = rotateRampTime;
        this.rotateP = rotateP;
        this.rotateD = rotateD;
        this.rotateMaxFeedback = rotateMaxFeedback;
        this.rotateTolerance = rotateTolerance;
        this.translateMaxVelocity = translateMaxVelocity;
        this.translateMaxAcceleration = translateMaxAcceleration;
        this.translateRampTime = translateRampTime;
        this.translateP = translateP;
        this.translateD = translateD;
        this.translateMaxFeedback = translateMaxFeedback;
        this.translateTolerance = translateTolerance;
    }

    /**
     * @return a builder for configuration
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Builder class for configuration
     */
    public static class Builder {

        String prefix = "SwerveAuto";
        double defaultRotateMaxVelocity = 720.0;
        double defaultRotateMaxAcceleration = 1440.0;
        double defaultRotateRampTime = 0.2;
        double defaultRotateP = 0.4;
        double defaultRotateD = 0.0;
        double defaultRotateMaxFeedback = 180.0;
        double defaultRotateTolerance = 1.0;
        double defaultTranslateMaxVelocity = 15.0;
        double defaultTranslateMaxAcceleration = 30.0;
        double defaultTranslateRampTime = 0.2;
        double defaultTranslateP = 2.0;
        double defaultTranslateD = 0.0;
        double defaultTranslateMaxFeedback = 5.0;
        double defaultTranslateTolerance = 2.0;

        /**
         * @param prefix prefix for fetching properties from
         * {@link edu.wpi.first.wpilibj.Preferences}
         * @return this builder
         */
        public Builder withPrefix(String prefix) {
            this.prefix = prefix;
            return this;
        }

        /**
         * @param rotateMaxVelocity default value for rotate max velocity in
         *                          degrees per second (will be overridden by
         *                          the corresponding preference if it already
         *                          exists)
         * @return this builder
         */
        public Builder withDefaultRotateMaxVelocity(double rotateMaxVelocity) {
            this.defaultRotateMaxVelocity = rotateMaxVelocity;
            return this;
        }

        /**
         * @param rotateMaxAcceleration default value for rotate max acceleration
         *                              in degrees per second squared (will be
         *                              overridden by the corresponding preference
         *                              if it already exists)
         * @return this builder
         */
        public Builder withDefaultRotateMaxAcceleration(double rotateMaxAcceleration) {
            this.defaultRotateMaxAcceleration = rotateMaxAcceleration;
            return this;
        }

        /**
         * @param rotateRampTime default value for rotate ramp time in seconds
         *                              (will be
         *                              overridden by the corresponding preference
         *                              if it already exists)
         * @return this builder
         */
        public Builder withDefaultRotateRampTime(double rotateRampTime) {
            this.defaultRotateRampTime = rotateRampTime;
            return this;
        }

        /**
         * @param rotateP default value for rotate proportional feedback (will
         *                be overridden by the corresponding preference if it
         *                already exists)
         * @return this builder
         */
        public Builder withDefaultRotateP(double rotateP) {
            this.defaultRotateP = rotateP;
            return this;
        }

        /**
         * @param rotateD default value for rotate derivative feedback (will
         *                be overridden by the corresponding preference if it
         *                already exists)
         * @return this builder
         */
        public Builder withDefaultRotateD(double rotateD) {
            this.defaultRotateD = rotateD;
            return this;
        }

        /**
         * @param rotateMaxFeedback default value for rotate max feedback in
         *                          degrees per second (will be overridden
         *                          by the corresponding preference if it
         *                          already exists)
         * @return this builder
         */
        public Builder withDefaultRotateMaxFeedback(double rotateMaxFeedback) {
            this.defaultRotateMaxFeedback = rotateMaxFeedback;
            return this;
        }

        /**
         * @param rotateTolerance default value for rotate tolerance in degrees
         *                        (will be overridden by the corresponding
         *                        preference if it already exists)
         * @return this builder
         */
        public Builder withDefaultRotateTolerance(double rotateTolerance) {
            this.defaultRotateTolerance = rotateTolerance;
            return this;
        }

        /**
         * @param translateMaxVelocity default value for translate max velocity
         *                             in feet per second (will be overridden by
         *                             the corresponding preference if it already
         *                             exists)
         * @return this builder
         */
        public Builder withDefaultTranslateMaxVelocity(double translateMaxVelocity) {
            this.defaultTranslateMaxVelocity = translateMaxVelocity;
            return this;
        }

        /**
         * @param translateMaxAcceleration default value for translate max
         *                                 acceleration in feet per second
         *                                 squared (will be overridden by the
         *                                 corresponding preference if it already
         *                                 exists)
         * @return this builder
         */
        public Builder withDefaultTranslateMaxAcceleration(double translateMaxAcceleration) {
            this.defaultTranslateMaxAcceleration = translateMaxAcceleration;
            return this;
        }

        /**
         * @param translateRampTime default value for translate ramp time in seconds
         *                              (will be
         *                              overridden by the corresponding preference
         *                              if it already exists)
         * @return this builder
         */
        public Builder withDefaultTranslateRampTime(double translateRampTime) {
            this.defaultTranslateRampTime = translateRampTime;
            return this;
        }

        /**
         * @param translateP default value for translate proportional feedback
         *                   (will be overridden by the corresponding preference
         *                   if it already exists)
         * @return this builder
         */
        public Builder withDefaultTranslateP(double translateP) {
            this.defaultTranslateP = translateP;
            return this;
        }

        /**
         * @param translateD default value for translate derivative feedback
         *                   (will be overridden by the corresponding preference
         *                   if it already exists)
         * @return this builder
         */
        public Builder withDefaultTranslateD(double translateD) {
            this.defaultTranslateD = translateD;
            return this;
        }

        /**
         * @param translateMaxFeedback default value for translate max feedback
         *                             in feet per second (will be overridden by
         *                             the corresponding preference if it already
         *                             exists)
         * @return this builder
         */
        public Builder withDefaultTranslateMaxFeedback(double translateMaxFeedback) {
            this.defaultTranslateMaxFeedback = translateMaxFeedback;
            return this;
        }

        /**
         * @param translateTolerance default value for translate tolerance in
         *                           inches (will be overridden by the
         *                           corresponding preference if it already
         *                           exists)
         * @return this builder
         */
        public Builder withDefaultTranslateTolerance(double translateTolerance) {
            this.defaultTranslateTolerance = translateTolerance;
            return this;
        }

        /**
         * @return a new {@link SwerveAutoPoseConfig}
         */
        public SwerveAutoPoseConfig build() {
            return new SwerveAutoPoseConfig(
                    Utils.pref(prefix+"/Rotate/MaxDegreesPerSec", defaultRotateMaxVelocity),
                    Utils.pref(prefix+"/Rotate/MaxAcceleration", defaultRotateMaxAcceleration),
                    Utils.pref(prefix+"/Rotate/RampTime", defaultRotateRampTime),
                    Utils.pref(prefix+"/Rotate/kP", defaultRotateP),
                    Utils.pref(prefix+"/Rotate/kD", defaultRotateD),
                    Utils.pref(prefix+"/Rotate/MaxFeedback", defaultRotateMaxFeedback),
                    Utils.pref(prefix+"/Rotate/Tolerance", defaultRotateTolerance),
                    Utils.pref(prefix+"/Translate/MaxFeetPerSec", defaultTranslateMaxVelocity),
                    Utils.pref(prefix+"/Translate/MaxAcceleration", defaultTranslateMaxAcceleration),
                    Utils.pref(prefix+"/Translate/RampTime", defaultTranslateRampTime),
                    Utils.pref(prefix+"/Translate/kP", defaultTranslateP),
                    Utils.pref(prefix+"/Translate/kD", defaultTranslateD),
                    Utils.pref(prefix+"/Translate/MaxFeedback", defaultTranslateMaxFeedback),
                    Utils.pref(prefix+"/Translate/Tolerance", defaultTranslateTolerance));
        }
    }
}
