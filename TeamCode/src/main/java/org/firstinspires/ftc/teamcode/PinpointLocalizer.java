package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;
import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {
    public static class Params {
        // legacy tick-based fields retained for compatibility with older code/configs
        @SuppressWarnings("unused")
        public double parYTicks = 0; // y position of the parallel encoder (in tick units)
        @SuppressWarnings("unused")
        public double perpXTicks = 0; // x position of the perpendicular encoder (in tick units)

        // New: pod positions in millimeters (set these to your measured values)
        // X pod (perpendicular) is -190 mm, Y pod (parallel) is -130 mm
        public double parYmm = -130.0; // Y (parallel) pod position in millimeters
        public double perpXmm = -160.0; // X (perp) pod position in millimeters
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver; // changed type to com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection;
    public final GoBildaPinpointDriver.EncoderDirection initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    // store encoder resolution (inches per tick) so telemetry can show it later
    private final double inPerTick;

    public PinpointLocalizer(HardwareMap hardwareMap, double _inPerTick, Pose2d initialPose) {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // store the provided inPerTick for later telemetry
        this.inPerTick = _inPerTick;

        // use the inPerTick passed into the constructor
        driver.setEncoderResolution(1.0 / this.inPerTick, DistanceUnit.INCH);
        // TODO: add in the correct offset values here for the pods to the computer
        //The Pinpoint computer needs to know where your tracking wheels are
        // relative to the center of rotation of your robot
        // (usually the physical center).•parYTicks (Parallel Offset):
        // This is the distance from the center of the robot to the parallel
        // (forward-facing) wheel along the Y-axis (left/right).
        // •Positive if the wheel is on the left side.
        // •Negative if the wheel is on the right side.
        // •perpXTicks (Perpendicular Offset):
        // This is the distance from the center of the robot to the perpendicular (strafe) wheel along the X-axis
        // (forward/backward).•Positive if the wheel is in front of the center.
        // •Negative if the wheel is behind the center.
        // NOTE: PARAMS.parYmm and PARAMS.perpXmm are in millimeters (set above).
        // The Pinpoint driver expects offsets in the units specified (inches here), so
        // convert millimeters to inches (1 in = 25.4 mm) before calling setOffsets.
        double parYInches = PARAMS.parYmm / 25.4;
        double perpXInches = PARAMS.perpXmm / 25.4;

        // driver.setOffsets expects (xOffset, yOffset) where x is forward/back (perp) and y is left/right (parallel)
        driver.setOffsets(perpXInches, parYInches, DistanceUnit.INCH);

        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);// This

        driver.resetPosAndIMU();

        // Runtime logging: record the Pinpoint configuration and status so you can inspect it later
        String cfg = String.format(Locale.US,
                "PINPOINT_CONFIG: perpXmm=%.3f, parYmm=%.3f, perpXin=%.4f, parYin=%.4f, inchesPerTick=%.6f, parDir=%s, perpDir=%s, status=%s",
                PARAMS.perpXmm, PARAMS.parYmm,
                perpXInches, parYInches,
                this.inPerTick,
                initialParDirection, initialPerpDirection,
                driver.getDeviceStatus());
        FlightRecorder.write("PINPOINT_CONFIG", cfg);

        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            Vector2d worldVelocity = new Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH));
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    // Public helper: add Pinpoint diagnostics to an on-driver Telemetry object (call from your OpMode)
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Pinpoint status", Objects.toString(driver.getDeviceStatus()));
        telemetry.addData("PerpX (mm)", PARAMS.perpXmm);
        telemetry.addData("ParY (mm)", PARAMS.parYmm);
        telemetry.addData("PerpX (in)", PARAMS.perpXmm / 25.4);
        telemetry.addData("ParY (in)", PARAMS.parYmm / 25.4);
        telemetry.addData("Inches per tick", this.inPerTick);

        // If the device is ready, show live Pinpoint-reported pose & velocities and the estimated pose
        GoBildaPinpointDriver.DeviceStatus status = driver.getDeviceStatus();
        if (status == GoBildaPinpointDriver.DeviceStatus.READY) {
            // Pinpoint device pose (inches)
            double pinX = driver.getPosX(DistanceUnit.INCH);
            double pinY = driver.getPosY(DistanceUnit.INCH);
            double pinHeadingRad = driver.getHeading(UnnormalizedAngleUnit.RADIANS);
            double pinHeadingDeg = Math.toDegrees(pinHeadingRad);

            telemetry.addData("PinX (in)", String.format(Locale.US, "%.3f", pinX));
            telemetry.addData("PinY (in)", String.format(Locale.US, "%.3f", pinY));
            telemetry.addData("PinHeading (deg)", String.format(Locale.US, "%.2f", pinHeadingDeg));

            // Pinpoint velocities
            double pinVelX = driver.getVelX(DistanceUnit.INCH);
            double pinVelY = driver.getVelY(DistanceUnit.INCH);
            double pinHeadingVelRad = driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
            double pinHeadingVelDeg = Math.toDegrees(pinHeadingVelRad);

            telemetry.addData("PinVelX (in/s)", String.format(Locale.US, "%.3f", pinVelX));
            telemetry.addData("PinVelY (in/s)", String.format(Locale.US, "%.3f", pinVelY));
            telemetry.addData("PinHeadingVel (deg/s)", String.format(Locale.US, "%.3f", pinHeadingVelDeg));

            // Estimated pose (after the internal transform)
            Pose2d est = getPose();
            telemetry.addData("EstX (in)", String.format(Locale.US, "%.3f", est.position.x));
            telemetry.addData("EstY (in)", String.format(Locale.US, "%.3f", est.position.y));
            telemetry.addData("EstHeading (deg)", String.format(Locale.US, "%.2f", Math.toDegrees(est.heading.toDouble())));
        }
    }
}
