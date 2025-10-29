package org.firstinspires.ftc.teamcode.Tutorial;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "HuskyLens AprilTag Dashboard", group = "Tutorial")
public class HuskyLensAprilTagDashboard extends LinearOpMode {

    private HuskyLens huskyLens;
    private final int READ_PERIOD = 1; // seconds
    private static final double FOCAL_LENGTH = 281; // pixels (tune for your setup)
    private static final double TAG_SIZE_INCHES = 6.5; // real tag size in inches

    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        dashboard = FtcDashboard.getInstance();

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        // Confirm communication
        if (!huskyLens.knock()) {
            telemetry.addData("Status", "HuskyLens not responding");
            telemetry.update();
            waitForStart();
            return;
        }

        // Select the AprilTag recognition algorithm
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addData("Status", "HuskyLens Ready");
        telemetry.addData("Note", "Make sure HuskyLens is in APRILTAG_RECOGNITION mode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Limit read frequency
            if (!rateLimit.hasExpired()) continue;
            rateLimit.reset();

            // Get detected tags
            HuskyLens.Block[] blocks = huskyLens.blocks();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("AprilTags Detected", blocks.length);

            telemetry.addData("AprilTags Detected", blocks.length);

            // Display data for each tag
            for (HuskyLens.Block tag : blocks) {
                double distance = calculateDistance(tag.width);

                // Send to Dashboard
                packet.put("Tag " + tag.id + " X", tag.x);
                packet.put("Tag " + tag.id + " Y", tag.y);
                packet.put("Tag " + tag.id + " Width", tag.width);
                packet.put("Tag " + tag.id + " Distance (in)", distance);

                // Driver Station telemetry
                telemetry.addLine(String.format(
                        "Tag %d | Center:(%d,%d) | W:%d | Dist:%.2f in",
                        tag.id, tag.x, tag.y, tag.width, distance));
            }

            // Send to FTC Dashboard
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();

            sleep(20);
        }
    }

    /**
     * Calculates distance from pixel width.
     */
    private double calculateDistance(double pixelWidth) {
        if (pixelWidth <= 0) return 0;
        return (FOCAL_LENGTH * TAG_SIZE_INCHES) / pixelWidth;
    }
}
