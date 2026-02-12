package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(group = "Primary", name = "AdvancedFieldCentric")
public class AdvancedFieldCentric extends LinearOpMode {

    // --- Hardware ---
    // Drive Train
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    // Mechanisms

    // Sensors
    private IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        while (!isStarted()) {
            telemetry.addData("Status", "Ready");
            telemetry.addData("Say Thanks To:","Edmond Zalkin (done in 2026");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            driveTrainFieldCentric();
            telemetryData();
        }
    }

    public void initHardware() {
        // --- Drive Motors --- Change as Necessary --- Pls don't use leftFront, it sounds stupid
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // --- IMU Setup --- Used for field centric drive
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientation));
    }


    // --- GAMEPAD 1: DRIVE & MECHANISMS ---
    public void driveTrainFieldCentric() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Deadzones
        if (Math.abs(y) < 0.05) y = 0;
        if (Math.abs(x) < 0.05) x = 0;
        if (Math.abs(rx) < 0.05) rx = 0;

        // Reset Yaw, change control as necessary
        if (gamepad1.back) imu.resetYaw();

        // Get Heading & Velocity
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double angularVel = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;

        // Predictive Heading (Lag Compensation)
        double correctedHeading = botHeading + (angularVel * 0.15);

        // Field Centric Math
        double rotX = x * Math.cos(-correctedHeading) - y * Math.sin(-correctedHeading);
        double rotY = x * Math.sin(-correctedHeading) + y * Math.cos(-correctedHeading);

        // Holding x (can be changed) makes the robot move at 50% speed
        double modifier = gamepad1.x ? 0.5 : 1.0;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        frontLeft.setPower(((rotY + rotX + rx) / denominator) * modifier);
        backLeft.setPower(((rotY - rotX + rx) / denominator) * modifier);
        frontRight.setPower(((rotY - rotX - rx) / denominator) * modifier);
        backRight.setPower(((rotY + rotX - rx) / denominator) * modifier);
    }
    // Telemetry

    public void telemetryData() {
        telemetry.addData("Heading", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        telemetry.update();
    }
}