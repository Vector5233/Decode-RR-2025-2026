package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Disabled
@TeleOp(name = "MainTeleOp", group = "Drive")
public class StarterTeleop extends OpMode {

    // Drivetrain
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Mechanisms
    private DcMotor collector;
    private DcMotor flywheel;
    private DcMotor liftLeft;
    private DcMotor liftRight;

    // Servos
    private Servo servo1;
    private Servo servo2;

    // Servo positions
    private static final double SERVO_OPEN = 1.0;
    private static final double SERVO_CLOSED = 0.0;

    @Override
    public void init() {

        // Drive motors
        frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft   = hardwareMap.dcMotor.get("backLeft");
        backRight  = hardwareMap.dcMotor.get("backRight");

        // Mechanisms
        collector  = hardwareMap.dcMotor.get("collector");
        flywheel   = hardwareMap.dcMotor.get("flywheel");
        liftLeft   = hardwareMap.dcMotor.get("L-Lift");
        liftRight  = hardwareMap.dcMotor.get("R-Lift");

        // Servos
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");

        // Set motor directions as needed
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Lift motors run together
        liftRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("TeleOp Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        //----------------------------
        // MECANUM DRIVE
        //----------------------------
        double y = -gamepad1.left_stick_y;     // Forward/back
        double x = gamepad1.left_stick_x * 1.1;// Strafing
        double rx = gamepad1.right_stick_x;    // Rotation

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double flPower = (y + x + rx) / denominator;
        double blPower = (y - x + rx) / denominator;
        double frPower = (y - x - rx) / denominator;
        double brPower = (y + x - rx) / denominator;

        frontLeft.setPower(flPower);
        backLeft.setPower(blPower);
        frontRight.setPower(frPower);
        backRight.setPower(brPower);


        //----------------------------
        // FLYWHEEL CONTROL (D-PAD)
        //----------------------------
        if (gamepad1.dpad_up) {
            flywheel.setPower(1.0);           // Full speed
        } else if (gamepad1.dpad_down) {
            flywheel.setPower(0.0);           // Stop
        }


        //----------------------------
        // COLLECTOR CONTROL (D-PAD)
        //----------------------------
        if (gamepad1.dpad_right) {
            collector.setPower(1.0);          // Intake
        } else if (gamepad1.dpad_left) {
            collector.setPower(-1.0);         // Reverse
        } else {
            collector.setPower(0.0);
        }


        //----------------------------
        // LIFT CONTROL (TRIGGERS)
        //----------------------------
        double liftUp = gamepad1.right_trigger;  // raise
        double liftDown = gamepad1.left_trigger; // lower

        double liftPower = liftUp - liftDown;

        liftLeft.setPower(liftPower);
        liftRight.setPower(liftPower);


        //----------------------------
        // SERVO CONTROL (BUTTONS)
        //----------------------------
        if (gamepad1.a) {             // open
            servo1.setPosition(SERVO_OPEN);
            servo2.setPosition(SERVO_OPEN);
        }
        if (gamepad1.b) {             // close
            servo1.setPosition(SERVO_CLOSED);
            servo2.setPosition(SERVO_CLOSED);
        }


        //----------------------------
        // TELEMETRY
        //----------------------------
        telemetry.addData("Drive", "FL %.2f  FR %.2f", flPower, frPower);
        telemetry.addData("Lift Power", liftPower);
        telemetry.addData("Flywheel", flywheel.getPower());
        telemetry.addData("Collector", collector.getPower());
        telemetry.update();
    }
}