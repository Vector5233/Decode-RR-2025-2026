package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

/**
 * PinpointLocalizer — wraps the GoBilda Pinpoint CPU to implement the
 * RoadRunner Localizer interface.
 *
 * HOW LOCALIZATION WORKS:
 *   Two passive "dead-wheel" pods (SwingArm style) roll along the floor and
 *   count encoder ticks as the robot moves.  The Pinpoint CPU reads both pods
 *   over I²C and fuses them with its onboard IMU to produce a continuous
 *   estimate of the robot's X position, Y position, and heading angle.
 *
 *   This class translates that estimate into the coordinate system RoadRunner
 *   expects (the "world" frame, measured in inches) and exposes it through
 *   three simple methods: setPose(), getPose(), and update().
 *
 * COORDINATE CONVENTIONS:
 *   +X  = robot's forward direction
 *   +Y  = robot's left direction
 *   +θ  = counter-clockwise rotation (standard math convention)
 *
 * POD PLACEMENT (measure from robot center of rotation):
 *   Parallel pod (Y pod):     runs forward/back, mounted left or right of center.
 *                             parYmm is positive if mounted LEFT, negative if RIGHT.
 *   Perpendicular pod (X pod): runs side to side, mounted in front or behind center.
 *                             perpXmm is positive if mounted IN FRONT, negative if BEHIND.
 */
@Config
public final class PinpointLocalizer implements Localizer {

    // -------------------------------------------------------------------------
    // PARAMS — pod offset positions.  Measure from the robot's center of
    // rotation to each pod wheel's contact point, in millimeters.
    // -------------------------------------------------------------------------
    public static class Params {
        /** Distance from robot center to the PARALLEL (forward/back) pod, left/right axis.
         *  Negative = pod is to the RIGHT of center. */
        public double parYmm  = -220.0;

        /** Distance from robot center to the PERPENDICULAR (strafe) pod, front/back axis.
         *  Negative = pod is BEHIND center. */
        public double perpXmm = -130.0;
    }

    public static Params PARAMS = new Params();

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------

    /** Direct reference to the Pinpoint driver — exposed so MecanumDrive can
     *  read raw telemetry and reset the heading. */
    public final GoBildaPinpointDriver driver;

    public final GoBildaPinpointDriver.EncoderDirection parDirection;
    public final GoBildaPinpointDriver.EncoderDirection perpDirection;

    // -------------------------------------------------------------------------
    // Pose bookkeeping
    // -------------------------------------------------------------------------

    // RoadRunner tracks pose as a chain of two transforms:
    //   txWorldPinpoint:  world → Pinpoint frame  (set by setPose, constant between resets)
    //   txPinpointRobot:  Pinpoint frame → robot  (updated every loop from the driver)
    //
    // getPose() returns txWorldPinpoint × txPinpointRobot = world → robot pose.
    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    private final double inPerTick;   // stored for telemetry display only

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------
    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {

        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        this.inPerTick = inPerTick;

        // Use GoBilda's built-in constant for Swingarm pods
        driver.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        // Convert pod offsets from millimeters to inches for the driver
        double parYIn  = PARAMS.parYmm  / 25.4;
        double perpXIn = PARAMS.perpXmm / 25.4;
        driver.setOffsets(perpXIn, parYIn, DistanceUnit.INCH);

        // Set encoder directions
        parDirection  = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        perpDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        driver.setEncoderDirections(parDirection, perpDirection);

        driver.resetPosAndIMU();

        txWorldPinpoint = initialPose;

        FlightRecorder.write("PINPOINT_CONFIG", String.format(Locale.US,
                "perpXmm=%.1f parYmm=%.1f perpXin=%.4f parYin=%.4f inPerTick=%.7f parDir=%s perpDir=%s status=%s",
                PARAMS.perpXmm, PARAMS.parYmm, perpXIn, parYIn, inPerTick,
                parDirection, perpDirection, driver.getDeviceStatus()));
    }
    // -------------------------------------------------------------------------
    // Localizer interface
    // -------------------------------------------------------------------------

    /**
     * Overrides the localizer's current pose estimate.
     * Call this at the start of autonomous to set the robot's known starting position.
     */
    @Override
    public void setPose(Pose2d pose) {
        // Adjust txWorldPinpoint so that the chain txWorldPinpoint × txPinpointRobot
        // equals the requested pose.
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    /**
     * Returns the latest pose estimate in the world frame (inches, radians).
     * Does NOT poll the hardware — call update() first each loop.
     */
    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    /**
     * Polls the Pinpoint for fresh encoder data, updates the pose estimate,
     * and returns the current robot velocity in the robot frame.
     * Must be called once per control loop iteration.
     */
    @Override
    public PoseVelocity2d update() {
        driver.update();


        if (driver.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {

            // Pinpoint reports position and velocity in the Pinpoint frame (world-aligned)
            txPinpointRobot = new Pose2d(
                    driver.getPosX(DistanceUnit.INCH),
                    driver.getPosY(DistanceUnit.INCH),
                    driver.getHeading(UnnormalizedAngleUnit.RADIANS));

            // Rotate the world-frame velocity into the robot frame for RoadRunner's controller
            Vector2d worldVelocity = new Vector2d(
                    driver.getVelX(DistanceUnit.INCH),
                    driver.getVelY(DistanceUnit.INCH));
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log())
                    .times(worldVelocity);

            return new PoseVelocity2d(robotVelocity,
                    driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }

        // Device not ready — return zero velocity so the robot stops safely
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    // -------------------------------------------------------------------------
    // Telemetry helper
    // -------------------------------------------------------------------------

    /**
     * Adds a full suite of Pinpoint diagnostics to the Driver Station telemetry.
     * Call from your OpMode after updatePoseEstimate() each loop.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Pinpoint Status",   driver.getDeviceStatus());
        telemetry.addData("ParY offset (mm)",  PARAMS.parYmm);
        telemetry.addData("PerpX offset (mm)", PARAMS.perpXmm);
        telemetry.addData("Inches per tick",   String.format(Locale.US, "%.7f", inPerTick));

        if (driver.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
            telemetry.addData("PinX (in)",           String.format(Locale.US, "%.3f", driver.getPosX(DistanceUnit.INCH)));
            telemetry.addData("PinY (in)",           String.format(Locale.US, "%.3f", driver.getPosY(DistanceUnit.INCH)));
            telemetry.addData("PinHeading (deg)",    String.format(Locale.US, "%.2f", Math.toDegrees(driver.getHeading(UnnormalizedAngleUnit.RADIANS))));
            telemetry.addData("PinVelX (in/s)",      String.format(Locale.US, "%.3f", driver.getVelX(DistanceUnit.INCH)));
            telemetry.addData("PinVelY (in/s)",      String.format(Locale.US, "%.3f", driver.getVelY(DistanceUnit.INCH)));

            Pose2d est = getPose();
            telemetry.addData("EstX (in)",       String.format(Locale.US, "%.3f", est.position.x));
            telemetry.addData("EstY (in)",       String.format(Locale.US, "%.3f", est.position.y));
            telemetry.addData("EstHeading (deg)", String.format(Locale.US, "%.2f", Math.toDegrees(est.heading.toDouble())));
        }
    }
}
