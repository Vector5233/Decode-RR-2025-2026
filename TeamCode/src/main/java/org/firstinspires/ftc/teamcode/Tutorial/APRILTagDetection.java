package org.firstinspires.ftc.teamcode.Tutorial;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "HuskyLens AprilTag Detection", group = "Tutorial")
public class APRILTagDetection extends LinearOpMode {

    private HuskyLens huskyLens;
    private final int READ_PERIOD = 1; // In seconds, how often to read from the HuskyLens

    // --- TUNING CONSTANTS ---
    // You must tune these values for your specific camera and setup
    private static final double FOCAL_LENGTH = 700; // Example: 700 pixels
    private static final double TAG_SIZE_INCHES = 2.0; // The actual physical size of the AprilTag (e.g., 2 inches)

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        // Set up a deadline to periodically read from the HuskyLens
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        // Check to make sure the HuskyLens is responding
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with HuskyLens");
            telemetry.update();
            // Wait for start or stop, then exit
            waitForStart();
            return;
        }

        // IMPORTANT: Select the APRILTAG_RECOGNITION algorithm on the HuskyLens device itself!
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addData(">>", "HuskyLens Initialized");
        telemetry.addData(">>", "Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            // Don't read too frequently to avoid I2C bus congestion
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            // The huskyLens.blocks() method returns a list of detected objects (AprilTags in this case)
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("AprilTags Detected", blocks.length);

            // Iterate through each detected tag
            for (HuskyLens.Block tag : blocks) {
                telemetry.addLine(String.format("\n==== (ID %d)", tag.id));
                telemetry.addLine(String.format("Center %d, %d (pixels)", tag.x, tag.y));

                // Calculate distance based on the perceived width of the tag
                double distance = calculateDistance(tag.width);
                telemetry.addLine(String.format("Estimated Distance: %.2f inches", distance));
            }

            telemetry.update();
            sleep(20); // Small sleep to share CPU
        }
    }

    /**
     * Estimates the distance to a target based on its perceived width in pixels.
     * @param pixelWidth The width of the detected object in pixels.
     * @return The estimated distance in inches.
     */
    private double calculateDistance(double pixelWidth) {
        // This formula is a common way to estimate distance in computer vision
        // You MUST tune the FOCAL_LENGTH and TAG_SIZE_INCHES constants for your setup.
        if (pixelWidth <= 0) {
            return 0; // Avoid division by zero
        }
        return (FOCAL_LENGTH * TAG_SIZE_INCHES) / pixelWidth;
    }
}
