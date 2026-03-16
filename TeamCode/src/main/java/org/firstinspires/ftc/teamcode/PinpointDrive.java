package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;
import java.util.Objects;

@Config
public final class PinpointDrive implements Localizer {

    /** Adjustable pod offsets (in millimeters) */
    @Config
    public static class Params {
        public double parYmm = -130.0;   // parallel wheel (left/right), negative = right side
        public double perpXmm = -190.0;  // perpendicular wheel (forward/back), negative = behind center
    }

    public static Params PARAMS = new Params();

    private final GoBildaPinpointDriver pinpoint;

    /** Transform between Pinpoint frame and robot frame */
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);
    private Pose2d txWorldPinpoint;

    private final double inchesPerTick;

    public PinpointDrive(HardwareMap hardwareMap, double _inchesPerTick, Pose2d initialPose) {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        this.inchesPerTick = _inchesPerTick;

        // Encoder resolution
        pinpoint.setEncoderResolution(1.0 / inchesPerTick, DistanceUnit.INCH);

        // Convert mm offsets → inches
        double parY = PARAMS.parYmm / 25.4;
        double perpX = PARAMS.perpXmm / 25.4;

        // Set wheel offsets
        pinpoint.setOffsets(perpX, parY, DistanceUnit.INCH);

        // Recommended directions for most FTC robots
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        pinpoint.resetPosAndIMU();

        txWorldPinpoint = initialPose;

        // Record configuration for debugging
        FlightRecorder.write("PINPOINT_CONFIG", String.format(Locale.US,
                "perpXmm=%.2f, parYmm=%.2f, perpXin=%.3f, parYin=%.3f, inPerTick=%.6f, status=%s",
                PARAMS.perpXmm, PARAMS.parYmm,
                perpX, parY,
                inchesPerTick,
                pinpoint.getDeviceStatus()
        ));
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
        pinpoint.update();

        if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {

            txPinpointRobot = new Pose2d(
                    pinpoint.getPosX(DistanceUnit.INCH),
                    pinpoint.getPosY(DistanceUnit.INCH),
                    pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS)
            );

            Vector2d vWorld = new Vector2d(
                    pinpoint.getVelX(DistanceUnit.INCH),
                    pinpoint.getVelY(DistanceUnit.INCH)
            );

            // Convert world velocity → robot frame
            Rotation2d rot = Rotation2d.fromDouble(-txPinpointRobot.heading.log());
            Vector2d vRobot = rot.times(vWorld);

            return new PoseVelocity2d(
                    vRobot,
                    pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
            );
        }

        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    /** Optional telemetry helper */
    public void addTelemetry(Telemetry telemetry) {

        telemetry.addData("Pinpoint Status", Objects.toString(pinpoint.getDeviceStatus()));
        telemetry.addData("Offsets (mm)", "perpX=%.1f, parY=%.1f",
                PARAMS.perpXmm, PARAMS.parYmm);

        if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {

            telemetry.addData("Pin Pos (in)",
                    "(%.2f, %.2f)",
                    pinpoint.getPosX(DistanceUnit.INCH),
                    pinpoint.getPosY(DistanceUnit.INCH));

            telemetry.addData("Pin Heading (deg)",
                    "%.1f",
                    Math.toDegrees(pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS)));

            telemetry.addData("Pin Vel (in/s)",
                    "(%.2f, %.2f)",
                    pinpoint.getVelX(DistanceUnit.INCH),
                    pinpoint.getVelY(DistanceUnit.INCH));

            Pose2d est = getPose();
            telemetry.addData("Estimated Pose",
                    "(%.2f, %.2f, %.1f°)",
                    est.position.x,
                    est.position.y,
                    Math.toDegrees(est.heading.toDouble()));
        }
    }
}
