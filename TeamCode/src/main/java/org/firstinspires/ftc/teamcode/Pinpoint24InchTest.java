package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Pinpoint 24 Inch Test", group="Calibration")
public class Pinpoint24InchTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // DIAGNOSTIC LOOP - check Pinpoint before running
        while (!isStarted() && !isStopRequested()) {
            drive.updatePoseEstimate();
            Pose2d p = drive.localizer.getPose();

            if (drive.localizer instanceof PinpointLocalizer) {
                PinpointLocalizer pl = (PinpointLocalizer) drive.localizer;
                telemetry.addData("Pinpoint Status", pl.driver.getDeviceStatus());
                telemetry.addData("PinX (in)", pl.driver.getPosX(
                        org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
                telemetry.addData("PinY (in)", pl.driver.getPosY(
                        org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
                telemetry.addData("Raw Par Ticks", pl.driver.getEncoderX());
                telemetry.addData("Raw Perp Ticks", pl.driver.getEncoderY());
                telemetry.addData("PinX (in)", pl.driver.getPosX(DistanceUnit.INCH));
            }

            telemetry.addData("Est X", p.position.x);
            telemetry.addData("Est Y", p.position.y);

            telemetry.update();
        }

        if (isStopRequested()) return;

        drive.localizer.setPose(new Pose2d(0, 0, 0));

        // Move slowly to reduce overshoot while diagnosing
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(24)
                        .build()
        );

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d p = drive.localizer.getPose();

            if (drive.localizer instanceof PinpointLocalizer) {
                PinpointLocalizer pl = (PinpointLocalizer) drive.localizer;
                telemetry.addData("Pinpoint Status", pl.driver.getDeviceStatus());
            }

            telemetry.addLine("--- TEST COMPLETE ---");
            telemetry.addData("Target X", 24.0);
            telemetry.addData("Actual X", p.position.x);
            telemetry.addData("Actual Y", p.position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(p.heading.toDouble()));

            telemetry.update();
        }
    }
}
