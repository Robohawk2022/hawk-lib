package robohawk.util.swerve;

import robohawk.util.HawkUtils;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Configuration properties for swerve teleop.
 */
public class SwerveTeleopConfig {

    /** Deadband for joystick input (default is 0.1) */
    public final DoubleSupplier deadband;

    /** Exponent for joystick input (default is 2.0) */
    public final DoubleSupplier exponent;

    /** Maximum translation speed in feet per second (default is 10.0) */
    public final DoubleSupplier maxTranslateSpeed;

    /** Maximum rotation speed in degrees per second (default is 360.0) */
    public final DoubleSupplier maxRotateSpeed;

    /** Turbo factor (default is 2.0) */
    public final DoubleSupplier turboFactor;

    /** Sniper factor (default is 0.5) */
    public final DoubleSupplier sniperFactor;

    /** Should sniper mode be applied to rotation? (default is false) */
    public final BooleanSupplier applySniperToRotate;

    /** Should we apply slew limiting? (default is false) */
    public final BooleanSupplier applySlew;

    /**
     * Slew rate limit (this is in "units per second) where units are in feet,
     * so a rate of 4.0 means you will hit 4 feet per second after 1 second
     */
    public final DoubleSupplier slewRate;

    /**
     * Should directions be interpreted relative to the driver (i.e. positive
     * X means "away from the driver", positive Y means "to the driver's
     * left")? (default is true)
     */
    public final BooleanSupplier driverRelative;

    private SwerveTeleopConfig(DoubleSupplier deadband,
                               DoubleSupplier exponent,
                               DoubleSupplier maxTranslateSpeed,
                               DoubleSupplier maxRotateSpeed,
                               DoubleSupplier turboFactor,
                               DoubleSupplier sniperFactor,
                               BooleanSupplier applySniperToRotate,
                               BooleanSupplier applySlew,
                               DoubleSupplier slewRate,
                               BooleanSupplier driverRelative) {
        this.deadband = deadband;
        this.exponent = exponent;
        this.maxTranslateSpeed = maxTranslateSpeed;
        this.maxRotateSpeed = maxRotateSpeed;
        this.turboFactor = turboFactor;
        this.sniperFactor = sniperFactor;
        this.applySniperToRotate = applySniperToRotate;
        this.applySlew = applySlew;
        this.slewRate = slewRate;
        this.driverRelative = driverRelative;
    }

    /**
     * @return a builder for configuration
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Builder class for configuration. When you use this, the property
     * values will be managed via WPILib Preferences. Preferences will be
     * registered with the following names:
     * <ul>
     *     <li><pre>${prefix}/Deadband</pre> - default 0.1</li>
     *     <li><pre>${prefix}/Exponent</pre> - default 2.0</li>
     *     <li><pre>${prefix}/MaxFeetPerSec</pre> - default 10.0</li>
     *     <li><pre>${prefix}/MaxDegreesPerSec</pre> - default 360.0</li>
     *     <li><pre>${prefix}/TurboFactor</pre> - default 2.0</li>
     *     <li><pre>${prefix}/SniperFactor</pre> - default 0.5</li>
     *     <li><pre>${prefix}/ApplySniperToRotate?</pre> - default false</li>
     *     <li><pre>${prefix}/ApplySlew?</pre> - default false</li>
     *     <li><pre>${prefix}/SlewRate</pre> - default 1.0</li>
     *     <li><pre>${prefix}/DriverRelative?</pre> - default true</li>
     * </ul>
     * See {@link HawkUtils#pref(String, double)} for some discussion of how
     * preferences work.
     */
    public static class Builder {

        String prefix = "SwerveTeleop";
        double defaultDeadband = 0.1;
        double defaultExponent = 2.0;
        double defaultMaxTranslateSpeed = 10.0;
        double defaultMaxRotateSpeed = 360.0;
        double defaultTurboFactor = 2.0;
        double defaultSniperFactor = 0.5;
        boolean defaultApplySniperToRotate = false;
        boolean defaultApplySlew = false;
        double defaultSlewRate = 1.0;
        boolean defaultDriverRelative = true;

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
         * @param deadband default value for deadband (will be overridden by
         *                 the corresponding preference if it already exists)
         * @return this builder
         */
        public Builder withDefaultDeadband(double deadband) {
            this.defaultDeadband = deadband;
            return this;
        }

        /**
         * @param exponent default value for exponent (will be overridden by
         *                 the corresponding preference if it already exists)
         * @return this builder
         */
        public Builder withDefaultExponent(double exponent) {
            this.defaultExponent = exponent;
            return this;
        }

        /**
         * @param maxTranslateSpeed default value for max translate speed, in
         *                          feet per second (will be overridden by the
         *                          corresponding preference if it already
         *                          exists)
         * @return this builder
         */
        public Builder withDefaultMaxTranslateSpeed(double maxTranslateSpeed) {
            this.defaultMaxTranslateSpeed = maxTranslateSpeed;
            return this;
        }

        /**
         * @param maxRotateSpeed default value for max rotation speed, in
         *                       degrees per second (will be overridden by
         *                       the corresponding preference if it already
         *                       exists)
         * @return this builder
         */
        public Builder withDefaultMaxRotateSpeed(double maxRotateSpeed) {
            this.defaultMaxRotateSpeed = maxRotateSpeed;
            return this;
        }

        /**
         * @param turboFactor default turbo factor (will be overridden by the
         *                    corresponding preference if it already
         *                    exists)
         * @return this builder
         */
        public Builder withDefaultTurboFactor(double turboFactor) {
            this.defaultTurboFactor = turboFactor;
            return this;
        }

        /**
         * @param sniperFactor default sniper factor (will be overridden by the
         *                     corresponding preference if it already
         *                     exists)
         * @return this builder
         */
        public Builder withDefaultSniperFactor(double sniperFactor) {
            this.defaultSniperFactor = sniperFactor;
            return this;
        }

        /**
         * @param slewRate default slew rate (will be overridden by the
         *                 corresponding preference if it already
         *                 exists)
         * @return this builder
         */
        public Builder withDefaultSlewRate(double slewRate) {
            this.defaultSlewRate = slewRate;
            return this;
        }

        /**
         * @param driverRelative default value (will be overridden by the
         *                       corresponding preference if it already
         *                       exists)
         * @return this builder
         */
        public Builder withDefaultDriverRelative(boolean driverRelative) {
            this.defaultDriverRelative = driverRelative;
            return this;
        }

        /**
         * @param applySlew default value (will be overridden by the
         *                  corresponding preference if it already
         *                  exists)
         * @return this builder
         */
        public Builder withDefaultApplySlew(boolean applySlew) {
            this.defaultApplySlew = applySlew;
            return this;
        }


        /**
         * @param applySniperToRotate default value (will be overridden by the
         *                            corresponding preference if it already
         *                            exists)
         * @return this builder
         */
        public Builder withDefaultApplySniperToRotate(boolean applySniperToRotate) {
            this.defaultApplySniperToRotate = applySniperToRotate;
            return this;
        }

        /**
         * @return a new {@link SwerveTeleopConfig}
         */
        public SwerveTeleopConfig build() {
            return new SwerveTeleopConfig(
                    HawkUtils.pref(prefix+"/Deadband", defaultDeadband),
                    HawkUtils.pref(prefix+"/Exponent", defaultExponent),
                    HawkUtils.pref(prefix+"/MaxFeetPerSec", defaultMaxTranslateSpeed),
                    HawkUtils.pref(prefix+"/MaxDegreesPerSec", defaultMaxRotateSpeed),
                    HawkUtils.pref(prefix+"/TurboFactor", defaultTurboFactor),
                    HawkUtils.pref(prefix+"/SniperFactor", defaultSniperFactor),
                    HawkUtils.pref(prefix+"/ApplySniperToRotate?", defaultApplySniperToRotate),
                    HawkUtils.pref(prefix+"/ApplySlew?", defaultApplySlew),
                    HawkUtils.pref(prefix+"/SlewRate", defaultSlewRate),
                    HawkUtils.pref(prefix+"/DriverRelative?", defaultDriverRelative));
        }
    }
}
