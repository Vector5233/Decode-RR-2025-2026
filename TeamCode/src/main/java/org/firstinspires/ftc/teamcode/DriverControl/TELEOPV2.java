package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(group = "Primary", name = "Field Centric - Final")
public class TELEOPV2 extends LinearOpMode {

    // --- SERVO POSITIONS ---
    private final double SERVO_POS_OPEN   = 0.75;
    private final double SERVO_POS_HOLD   = 0.50;
    private final double SERVO_POS_CLOSED = 0.25;

    // --- Hardware ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx flywheel;
    private DcMotor collector;
    private Servo servo1, servo2;
    private IMU imu;

    // --- State Variables ---
    private double targetFlywheelPower = 0.0;
    private double currentFlywheelPower = 0.0;
    private final double FLYWHEEL_RAMP = 0.02;
    private final double FLYWHEEL_ON_POWER = 1.00;
    private double collectorPower = 0.0;

    private boolean lastRB = false, lastLB = false, lastA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        while (!isStarted()) {
            telemetry.addData("Status", "Field Centric Ready");
            telemetry.addData("Note", "Point robot AWAY and press Start");
            telemetry.addData("Reset Yaw", "Press Options/Back button");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            driveTrainFieldCentric();
            flywheels();
            servos();
            collector();
            telemetryData();
        }
    }

    public void initHardware() {
        // Motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // IMU Setup for Field Centric
        imu = hardwareMap.get(IMU.class, "imu");
        // ADJUST THESE based on how your hub is mounted:
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        collector = hardwareMap.get(DcMotor.class, "collector");

        servo1 = hardwareMap.get(Servo.class, "servoOne");
        servo2 = hardwareMap.get(Servo.class, "servoTwo");
        servo2.setDirection(Servo.Direction.REVERSE);
    }

    public void driveTrainFieldCentric() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Joystick Deadzone
        if (Math.abs(y) < 0.05) y = 0;
        if (Math.abs(x) < 0.05) x = 0;
        if (Math.abs(rx) < 0.05) rx = 0;

        if (gamepad1.back) imu.resetYaw();

        // 1. Get current heading
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // 2. GET ANGULAR VELOCITY (The "Secret Sauce" for accuracy)
        // This tells us how fast the robot is currently rotating
        double angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;

        // 3. PREDICTIVE HEADING
        // We add a small "look-ahead" based on velocity to compensate for loop lag
        double correctedHeading = botHeading + (angularVelocity * 0.015); // Adjust 0.015 based on testing

        // Field Centric Math using the corrected heading
        double rotX = x * Math.cos(-correctedHeading) - y * Math.sin(-correctedHeading);
        double rotY = x * Math.sin(-correctedHeading) + y * Math.cos(-correctedHeading);

        double modifier = gamepad1.x ? 0.5 : 1.0;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        frontLeft.setPower(((rotY + rotX + rx) / denominator) * modifier);
        backLeft.setPower(((rotY - rotX + rx) / denominator) * modifier);
        frontRight.setPower(((rotY - rotX - rx) / denominator) * modifier);
        backRight.setPower(((rotY + rotX - rx) / denominator) * modifier);
    }

    public void flywheels() {
        if (gamepad1.a && !lastA) {
            targetFlywheelPower = (targetFlywheelPower == 0) ? FLYWHEEL_ON_POWER : 0;
        }
        lastA = gamepad1.a;

        if (Math.abs(currentFlywheelPower - targetFlywheelPower) > FLYWHEEL_RAMP) {
            currentFlywheelPower += (targetFlywheelPower > currentFlywheelPower) ? FLYWHEEL_RAMP : -FLYWHEEL_RAMP;
        } else {
            currentFlywheelPower = targetFlywheelPower;
        }
        flywheel.setPower(currentFlywheelPower);
    }

    public void collector() {
        if (gamepad1.right_bumper && !lastRB) {
            collectorPower = (collectorPower == 1.0) ? 0.0 : 1.0;
        }
        lastRB = gamepad1.right_bumper;

        if (gamepad1.left_bumper && !lastLB) {
            collectorPower = (collectorPower == -1.0) ? 0.0 : -1.0;
        }
        lastLB = gamepad1.left_bumper;

        collector.setPower(collectorPower);
    }

    public void servos() {
        if (gamepad1.right_trigger > 0.25) {
            servo1.setPosition(SERVO_POS_OPEN);
            servo2.setPosition(SERVO_POS_OPEN);
        }
        else if (gamepad1.left_trigger > 0.25) {
            servo1.setPosition(SERVO_POS_CLOSED);
            servo2.setPosition(SERVO_POS_CLOSED);
        }
        else if (gamepad1.y) {
            servo1.setPosition(SERVO_POS_HOLD);
            servo2.setPosition(SERVO_POS_HOLD);
        }
    }

    public void telemetryData() {
        telemetry.addData("Heading (Deg)", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        telemetry.addData("Flywheel", currentFlywheelPower);
        telemetry.addData("Intake", collectorPower);
        telemetry.update();
    }
}