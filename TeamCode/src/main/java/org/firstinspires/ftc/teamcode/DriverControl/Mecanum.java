package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mecanum", group = "TeleOp")
public class Mecanum extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Get gamepad inputs (doubles)
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y; // Invert Y because up is negative
            double turn = gamepad1.right_stick_x;

            // 2. Calculate power for each motor
            double flPower = forward + strafe + turn;
            double blPower = forward - strafe + turn;
            double frPower = forward - strafe - turn;
            double brPower = forward + strafe - turn;

            // 3. Normalize the values
            // This ensures no motor is told to go over 1.0, preserving the ratio of power
            double max = Math.max(Math.abs(flPower), Math.abs(blPower));
            max = Math.max(max, Math.abs(frPower));
            max = Math.max(max, Math.abs(brPower));

            if (max > 1.0) {
                flPower /= max;
                blPower /= max;
                frPower /= max;
                brPower /= max;
            }

            // 4. Set motor powers
            frontLeft.setPower(flPower);
            backLeft.setPower(blPower);
            frontRight.setPower(frPower);
            backRight.setPower(brPower);

            // Optional: Add telemetry for debugging
            telemetry.addData("Motors", "FL: %.2f, FR: %.2f", flPower, frPower);
            telemetry.update();
        }
    }

    private void initHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Most Mecanum bots need one side reversed
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Helps the robot stop faster
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}