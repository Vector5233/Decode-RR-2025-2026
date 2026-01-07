package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "Primary", name = "Robot Final - Standard Mode")
public class TELEOPV2 extends LinearOpMode {

    // --- SERVO POSITIONS (Standard Mode) ---
    private final double SERVO_POS_OPEN   = 0.75; // Press A
    private final double SERVO_POS_HOLD   = 0.50; // Press Y (Shared with Flywheel)
    private final double SERVO_POS_CLOSED = 0.25; // Press B

    // --- Declare Hardware ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx flywheel;
    private DcMotor collector;
    private Servo servo1, servo2;

    // --- State Variables for Toggles ---
    private double targetFlywheelPower = 0.0;
    private double currentFlywheelPower = 0.0;
    private final double FLYWHEEL_RAMP = 0.02;
    private final double FLYWHEEL_ON_POWER = 1.00;

    private double collectorPower = 0.0;

    // Edge Detection (Prevents rapid-fire toggling)
    private boolean lastRB = false;
    private boolean lastLB = false;
    private boolean lastA  = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        while (!isStarted()) {
            telemetry.addData("Status", "Standard Mode Ready");
            telemetry.addData("Intake", "RB/LB Toggles");
            telemetry.addData("Servos", "A=Open, B=Closed, Y=Hold");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            driveTrain();
            flywheels(); // Y Button Toggle
            servos();    // A, B, Y Buttons
            collector(); // RB/LB Toggle
            telemetryData();
        }
    }

    public void initHardware() {
        // Drive Motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Subsystems
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        collector = hardwareMap.get(DcMotor.class, "collector");

        // Servos (Reversed for Mirroring)
        servo1 = hardwareMap.get(Servo.class, "servoOne");
        servo2 = hardwareMap.get(Servo.class, "servoTwo");
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);

        // Initial Safe Position
        servo1.setPosition(SERVO_POS_HOLD);
        servo2.setPosition(SERVO_POS_HOLD);
    }

    public void driveTrain() {
        double modifier = gamepad1.x ? 0.5 : 1.0;
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        frontLeft.setPower((drive + strafe + turn) * modifier);
        backLeft.setPower((drive - strafe + turn) * modifier);
        frontRight.setPower((drive - strafe - turn) * modifier);
        backRight.setPower((drive + strafe - turn) * modifier);
    }

    public void flywheels() {
        // Toggle Flywheel with Y
        if (gamepad1.a && !lastA) {
            targetFlywheelPower = (targetFlywheelPower == 0) ? FLYWHEEL_ON_POWER : 0;
        }
        lastA = gamepad1.a;

        // Smooth Ramp
        if (Math.abs(currentFlywheelPower - targetFlywheelPower) > FLYWHEEL_RAMP) {
            currentFlywheelPower += (targetFlywheelPower > currentFlywheelPower) ? FLYWHEEL_RAMP : -FLYWHEEL_RAMP;
        } else {
            currentFlywheelPower = targetFlywheelPower;
        }
        flywheel.setPower(currentFlywheelPower);
    }

    public void collector() {
        // Toggle Forward with Right Bumper
        if (gamepad1.right_bumper && !lastRB) {
            collectorPower = (collectorPower == 1.0) ? 0.0 : 1.0;
        }
        lastRB = gamepad1.right_bumper;

        // Toggle Reverse with Left Bumper
        if (gamepad1.left_bumper && !lastLB) {
            collectorPower = (collectorPower == -1.0) ? 0.0 : -1.0;
        }
        lastLB = gamepad1.left_bumper;

        collector.setPower(collectorPower);
    }

    public void servos() {
        // Standard servos stay where they are put.
        // We use if/else if so that if no button is pressed,
        // they just keep their previous position.
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
        telemetry.addData("Intake State", (collectorPower == 0) ? "OFF" : (collectorPower > 0) ? "FORWARD" : "REVERSE");
        telemetry.addData("Flywheel Power", currentFlywheelPower);
        telemetry.addData("Servo Pos", "%.2f", servo1.getPosition());
        telemetry.update();
    }
}