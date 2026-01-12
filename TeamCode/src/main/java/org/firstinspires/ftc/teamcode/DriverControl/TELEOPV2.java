package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(group = "Primary", name = "Final Robot Control")
public class TELEOPV2 extends LinearOpMode {

    // --- SERVO POSITIONS ---
    private final double SERVO_POS_OPEN   = 0.75;
    private final double SERVO_POS_HOLD   = 0.50;
    private final double SERVO_POS_CLOSED = 0.25;

    // --- Hardware ---
    // Drive Train
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    // Mechanisms
    private DcMotorEx flywheel;
    private DcMotor collector;
    private DcMotorEx leftLift, rightLift; // <--- NEW LIFT MOTORS
    private Servo servo1, servo2;
    // Sensors
    private IMU imu;

    // --- State Variables ---
    private double targetFlywheelPower = 0.0;
    private double currentFlywheelPower = 0.0;
    private final double FLYWHEEL_RAMP = 0.02;
    private final double FLYWHEEL_ON_POWER = 1.00;

    private double collectorPower = 0.0;

    // Toggles
    private boolean lastRB = false, lastLB = false, lastA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        while (!isStarted()) {
            telemetry.addData("Status", "Ready");
            telemetry.addData("Lift", "Gamepad 2 Triggers");
            telemetry.addData("Drive", "Field Centric");
            telemetry.addData("You got this!","true");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            driveTrainFieldCentric();
            liftLogic(); // <--- NEW LIFT FUNCTION
            flywheels();
            servos();
            collector();
            telemetryData();
        }
    }

    public void initHardware() {
        // --- Drive Motors ---
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // --- Lift Motors ---
        // We assume one needs to be reversed so they both move UP together.
        // If they fight each other, change REVERSE to FORWARD here.
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Other Hardware ---
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        collector = hardwareMap.get(DcMotor.class, "collector");

        servo1 = hardwareMap.get(Servo.class, "servoOne");
        servo2 = hardwareMap.get(Servo.class, "servoTwo");
        servo2.setDirection(Servo.Direction.REVERSE);

        // --- IMU Setup ---
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientation));
    }

    // --- GAMEPAD 2: LIFT CONTROL ---
    public void liftLogic() {
        // Right Trigger = Up (Positive)
        // Left Trigger = Down (Negative)
        double liftPower = gamepad2.right_trigger - gamepad2.left_trigger;

        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);
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

        // Reset Yaw
        if (gamepad1.back) imu.resetYaw();

        // Get Heading & Velocity
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double angularVel = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;

        // Predictive Heading (Lag Compensation)
        double correctedHeading = botHeading + (angularVel * 0.15);

        // Field Centric Math
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
        telemetry.addData("Lift Power", "%.2f", leftLift.getPower());
        telemetry.addData("Odo Parallel", frontLeft.getCurrentPosition());
        telemetry.addData("Odo Perp", frontRight.getCurrentPosition());
        telemetry.addData("Heading", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        telemetry.update();
    }
}