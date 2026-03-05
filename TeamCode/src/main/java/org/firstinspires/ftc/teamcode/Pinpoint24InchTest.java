package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Pinpoint 24 Inch Test", group="Calibration")
public class Pinpoint24InchTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Pose
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        if (isStopRequested()) return;

        // 1. Reset the localizer and the drive pose to 0,0,0
        drive.localizer.setPose(new Pose2d(0, 0, 0));

        // 2. Execute the move (Forward 24 inches)
        // Actions.runBlocking waits for the movement to finish before continuing
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(24)
                        .build()
        );

        // 3. TELEMETRY LOOP
        // After the robot stops, this loop keeps running so you can look
        // at the phone and see where the robot *thinks* it ended up.
        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            // In RR 1.0, we access position via .position and heading via .heading
            Pose2d p = drive.localizer.getPose();
            //Pose2d p = drive.updatePoseEstimate().value();
            telemetry.addLine("--- TEST COMPLETE ---");
            telemetry.addData("Target X", 24.0);
            telemetry.addData("Actual X (RR Estimate)", p.position.x);
            telemetry.addData("Actual Y (RR Estimate)", p.position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(p.heading.toDouble()));
            telemetry.addLine("\nIf 'Actual X' is NOT 24, adjust inPerTick in MecanumDrive.");
            telemetry.update();
        }
    }
}