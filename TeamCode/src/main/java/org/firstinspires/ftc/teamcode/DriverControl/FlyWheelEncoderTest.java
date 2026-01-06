package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Test: Flywheel Encoder", group = "Test")
public class FlyWheelEncoderTest extends LinearOpMode {

    private DcMotorEx flywheelMotor;

    @Override
    public void runOpMode() {
        // --- Initialization ---
        try {
            flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");

            // Stop and reset the encoder to 0. This is crucial.
            flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set the motor to run using the encoder, which allows us to read its position.
            // It won't be powered unless you set a power level, so it's safe to spin by hand.
            flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Status", "Initialization Successful");
            telemetry.addData(">", "Ready to start");

        } catch (Exception e) {
            flywheelMotor = null;
            telemetry.addData("Status", "Error: Flywheel motor " +
                    ".  or not found!");
        }
        telemetry.update();

        waitForStart();

        // --- Main Loop ---
        while (opModeIsActive()) {
            if (flywheelMotor != null) {
                // Read the current encoder position
                int currentPosition = flywheelMotor.getCurrentPosition();

                telemetry.addLine("Spin the flywheel motor by hand to find its ticks per revolution.");
                telemetry.addLine();
                telemetry.addData("Current Encoder Ticks", currentPosition);
                telemetry.addLine();
                telemetry.addLine("Press (A) to reset the encoder count to 0.");

                // Allow resetting the encoder count during the OpMode
                if (gamepad1.a) {
                    flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Switch back to reading mode
                }

            } else {
                telemetry.addData("Error", "Motor not initialized. Check configuration.");
            }

            telemetry.update();
        }
    }
}
