package robohawk.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import robohawk.util.HawkUtils;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Supplier for ChassisSpeeds that processes stick input and provides
 * several useful options that we wind up using year over year:
 * <ul>
 *
 *     <li>Max translate/rotate speed (to convert 0.0 - 1.0 values to
 *     a valid field speed)</li>
 *
 *     <li>Deadband (highly recommended for joysticks that don't
 *     report 0.0 values when centered)</li>
 *
 *     <li>Exponent (lore has it that squaring or cubing small
 *     input values gives better control)</li>
 *
 *     <li>"Turbo" and "Sniper" Factors for translation (allows you to
 *     implement stuff like a simple "double my top speed" control)</li>
 *
 *     <li>One-sided slew rate limiting for translation (i.e.
 *     don't accelerate from 0 instantaneously, but "snap back"
 *     to 0 if they release the joystick)</li>
 *
 *     <li>"Driver relative" speeds, so that pushing the joystick away
 *     from the driver always sends the robot away from the driver
 *     (see {@link HawkUtils#convertFromDriverRelative(Rotation2d, ChassisSpeeds)})</li>
 *
 *     <li>Publishing mode, input and speed to SmartDashboard</li>
 *
 * </ul>
 */
public class SwerveTeleopSpeedSupplier implements Supplier<ChassisSpeeds> {

    private enum Mode {
        TURBO,
        SNIPER,
        NONE
    }

    final SwerveTeleopConfig config;
    final DoubleSupplier x;
    final DoubleSupplier y;
    final DoubleSupplier omega;
    final BooleanSupplier turboTrigger;
    final BooleanSupplier sniperTrigger;
    final Supplier<Rotation2d> headingGetter;
    SlewRateLimiter limiterX;
    SlewRateLimiter limiterY;
    double inX;
    double inY;
    double inO;
    double lastX;
    double lastY;
    double lastOmega;

    /**
     * Creates a {@link SwerveTeleopSpeedSupplier}. Supports optionally
     * publishing current mode, input and speed to the dashboard for
     * debugging.
     *
     * @param config configuration (required)
     * @param x supplies X translation ratio (-1.0, 1.0) (required)
     * @param y supplies Y translation ratio (-1.0, 1.0) (required)
     * @param omega supplies rotation ratio (-1.0, 1.0) (required)
     * @param turboTrigger enables "turbo mode"
     * @param sniperTrigger enables "sniper mode"
     * @param headingGetter getter for the robot's heading (required)
     * @param dashboardName name to publish under (if null, don't publish)
     * @throws IllegalArgumentException if required parameters are null
     */
    public SwerveTeleopSpeedSupplier(SwerveTeleopConfig config,
                                     DoubleSupplier x,
                                     DoubleSupplier y,
                                     DoubleSupplier omega,
                                     BooleanSupplier turboTrigger,
                                     BooleanSupplier sniperTrigger,
                                     Supplier<Rotation2d> headingGetter,
                                     String dashboardName) {

        Objects.requireNonNull(config);
        Objects.requireNonNull(config);
        Objects.requireNonNull(x);
        Objects.requireNonNull(y);
        Objects.requireNonNull(omega);
        if (turboTrigger == null) {
            turboTrigger = () -> false;
        }
        if (sniperTrigger == null) {
            sniperTrigger = () -> false;
        }

        this.config = config;
        this.x = x;
        this.y = y;
        this.omega = omega;
        this.turboTrigger = turboTrigger;
        this.sniperTrigger = sniperTrigger;
        this.headingGetter = headingGetter;
        this.lastX = Double.NaN;
        this.lastY = Double.NaN;
        this.lastOmega = Double.NaN;

        if (dashboardName != null) {
            SmartDashboard.putData(dashboardName, builder -> {
                builder.addStringProperty("Mode", () -> getMode().toString(), null);
                builder.addDoubleProperty("InputX", () -> inX, null);
                builder.addDoubleProperty("InputY", () -> inY, null);
                builder.addDoubleProperty("InputOmega", () -> inO, null);
                builder.addDoubleProperty("SpeedX", () -> lastX, null);
                builder.addDoubleProperty("SpeedY", () -> lastY, null);
                builder.addDoubleProperty("SpeedOmega", () -> lastOmega, null);
            });
        }
    }

    private Mode getMode() {
        if (sniperTrigger.getAsBoolean()) {
            return Mode.SNIPER;
        } else if (turboTrigger.getAsBoolean()) {
            return Mode.TURBO;
        } else {
            return Mode.NONE;
        }
    }

    /**
     * @return calculates chassis speeds based on current controller input
     */
    @Override
    public ChassisSpeeds get() {

        // get input from the joystick
        inX = x.getAsDouble();
        inY = y.getAsDouble();
        inO = omega.getAsDouble();

        // "condition" the values with deadband etc.
        lastX = conditionInput(inX);
        lastY = conditionInput(inY);
        lastOmega = conditionInput(inO);

        // ensure that the point defined by (x, y) lies on the unit
        // circle - when we scale them by the maximum translate speed
        // this will prevent us from shooting off too fast at an angle
        double d = Math.hypot(lastX, lastY);
        if (d > 1.0) {
            lastX /= d;
            lastY /= d;
        }

        // convert to speeds
        double mt = config.maxTranslateSpeed.getAsDouble();
        lastX *= mt;
        lastY *= mt;
        lastOmega *= config.maxRotateSpeed.getAsDouble();

        // update slew settings and apply slew rate limiting if necessary
        checkSlew();
        if (limiterX != null) {
            lastX = slewLimit(lastX, limiterX);
        }
        if (limiterY != null) {
            lastY = slewLimit(lastY, limiterY);
        }

        switch (getMode()) {

            // if we're in sniper mode, we might want to slow down the rotation too
            case SNIPER:
                double sf = config.sniperFactor.getAsDouble();
                lastX *= sf;
                lastY *= sf;
                if (config.applySniperToRotate.getAsBoolean()) {
                    lastOmega *= sf;
                }
                break;

            // if we're in turbo mode, we only change translation
            case TURBO:
                double tf = config.turboFactor.getAsDouble();
                lastX *= tf;
                lastY *= tf;
                break;

            // nothing to do here
            case NONE:
                break;

        }

        ChassisSpeeds speeds = new ChassisSpeeds(
                Units.feetToMeters(lastX),
                Units.feetToMeters(lastY),
                Math.toRadians(lastOmega));

        // convert from driver-relative speeds
        if (config.driverRelative.getAsBoolean()) {
            speeds = HawkUtils.convertFromDriverRelative(
                    headingGetter.get(),
                    speeds);
        }

        return speeds;
    }

    // make sure that we're either applying or not applying slew rate limiting
    // depending on the configuration property
    private void checkSlew() {
        if (config.applySlew.getAsBoolean()) {
            if (limiterX == null) {
                double slew = config.slewRate.getAsDouble();
                limiterX = new SlewRateLimiter(slew);
                limiterY = new SlewRateLimiter(slew);
                HawkUtils.log("[swerve-teleop] enabling slew rate limiting @ %.2f", slew);
            }
        } else if (limiterX != null) {
            limiterX = null;
            limiterY = null;
            HawkUtils.log("[swerve-teleop] disabling slew rate limiting");
        }
    }

    /*
     * Applies slew limiting, but only when we're moving. If the desired speed
     * is 0, we will simply stop and reset the slew limit. This is to keep
     * manual targeting accurate.
     */
    private double slewLimit(double value, SlewRateLimiter limiter) {
        if (value == 0.0) {
            limiter.reset(0.0);
        } else {
            value = limiter.calculate(value);
        }
        return value;
    }

    /*
     * Conditions joystick input with deadband and exponent
     */
    private double conditionInput(double input) {
        input = MathUtil.clamp(input, -1.0, 1.0);
        input = MathUtil.applyDeadband(input, config.deadband.getAsDouble());
        input = Math.copySign(Math.pow(input, config.exponent.getAsDouble()), input);
        return input;
    }

    /**
     * Creates a {@link SwerveTeleopSpeedSupplier} with the default mappings
     * used by Team 3373 year-over-year:
     * <ul>
     *     <li>Left stick is translation (forward is +X, left is +Y)</li>
     *     <li>Right stick is rotation (left is +omega)</li>
     *     <li>Hold left trigger for turbo mode</li>
     *     <li>Hold right trigger for sniper mode</li>
     * </ul>
     *
     * @param config configuration (required)
     * @param headingGetter getter for robot's current heading (required)
     * @param controller controller (required)
     * @param dashboardName name to publish under (if null, don't publish)
     * @throws IllegalArgumentException if required parameters are null
     * @return a new {@link SwerveTeleopSpeedSupplier}
     */
    public static SwerveTeleopSpeedSupplier defaultControllerMapping(
            SwerveTeleopConfig config,
            Supplier<Rotation2d> headingGetter,
            CommandXboxController controller,
            String dashboardName) {

        Objects.requireNonNull(config);
        Objects.requireNonNull(headingGetter);
        Objects.requireNonNull(controller);

        // pushing right or forward on the joystick results in negative values,
        // so we invert them before using them
        DoubleSupplier leftX = () -> -controller.getLeftX();
        DoubleSupplier leftY = () -> -controller.getLeftY();
        DoubleSupplier rightX = () -> -controller.getRightX();

        // triggers controller sniper/turbo behavior
        BooleanSupplier sniperTrigger = () -> controller.getLeftTriggerAxis() > 0.5;
        BooleanSupplier turboTrigger = () -> controller.getRightTriggerAxis() > 0.5;

        return new SwerveTeleopSpeedSupplier(
                config,
                leftX,
                leftY,
                rightX,
                turboTrigger,
                sniperTrigger,
                headingGetter,
                dashboardName);
    }
}
