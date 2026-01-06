package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo; // Changed from CRServo to Servo
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
/**Configuration file

 * Drive train motors will called - we many want to make a diveMotor class later - Chad this would be your input

 * motor names in the configuration: on Robot Controller: this is where you name the motors in the app

 * frontLeftMotor

 * frontRightMotor

 * backLeftMotor

 * backRightMotor

 * flyWheel

 * collector

 * leftLift

 * rightLift

 * servoOne

 * servoTwo

 */
@TeleOp(group = "Primary", name = "Pick This One")
public class TELEOPV2 extends LinearOpMode {

    // --- Declare Motors ---
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotorEx flywheel;
    private DcMotor collector;

    // --- Declare Standard Servos ---
    // Changed from CRServo to Servo (Standard "Single Mode" Servos)
    private Servo servo1;
    private Servo servo2;

    // --- Constants & Variables ---
    // Update TICKS_PER_REV for your specific motor (e.g., GoBilda 5202: 537.7, Rev HD Hex: 28)
    private final double TICKS_PER_REV = 28;

    // Servo Positions (Tune these values between 0.0 and 1.0)
    private final double SERVO_POS_A = 1.0;
    private final double SERVO_POS_B = 0.0;

    private double targetFlywheelPower = 0.0;
    private double currentFlywheelPower = 0.0;
    private final double FLYWHEEL_RAMP = 0.02;
    private final double FLYWHEEL_ON_POWER = 1.00;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        while (!isStarted()) {
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Servo Note", "A=Pos 1.0, B=Pos 0.0");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            driveTrain();
            flywheels(); // Handles Flywheel AND Servos
            collector();
            telemetryData();
        }
    }

    public void initHardware() {
        initDriveTrain();
        initFlywheelMotor();
        initServos();
        initCollector();
    }

    public void initDriveTrain() {
        try {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
            frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
            backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
            backRight = hardwareMap.get(DcMotor.class, "backRightMotor");

            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addData("Error", "Drive Train Init Failed");
        }
    }

    public void driveTrain() {
        double modifier = gamepad1.x ? 0.5 : 1.0;

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double frontLeftPower = drive + strafe + turn;
        double backLeftPower = drive - strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backRightPower = drive + strafe - turn;

        double[] powers = {frontLeftPower, backLeftPower, frontRightPower, backRightPower};
        double max = 1.0;

        for (double p : powers) {
            if (Math.abs(p) > max) {
                max = Math.abs(p);
            }
        }

        frontLeft.setPower(frontLeftPower / max * modifier);
        backLeft.setPower(backLeftPower / max * modifier);
        frontRight.setPower(frontRightPower / max * modifier);
        backRight.setPower(backRightPower / max * modifier);
    }

    public void flywheels() {
        // --- Flywheel Logic (Single Motor) ---
        boolean currRightBumper = gamepad1.right_bumper;

        // Toggle Logic
        if (currRightBumper && !lastRightBumper) {
            if (Math.abs(targetFlywheelPower) < 0.01) {
                targetFlywheelPower = FLYWHEEL_ON_POWER;
            } else {
                targetFlywheelPower = 0.0;
            }
        }
        lastRightBumper = currRightBumper;

        // Ramp Logic
        if (Math.abs(currentFlywheelPower - targetFlywheelPower) > FLYWHEEL_RAMP) {
            if (currentFlywheelPower < targetFlywheelPower)
                currentFlywheelPower += FLYWHEEL_RAMP;
            else currentFlywheelPower -= FLYWHEEL_RAMP;
        } else {
            currentFlywheelPower = targetFlywheelPower;
        }

        if (flywheel != null) {
            flywheel.setPower(currentFlywheelPower);
        }

        // --- Servo Logic (Standard Position) ---
        if (gamepad1.a) {
            // Press A to go to Position 1 (e.g., 1.0)
            if (servo1 != null) servo1.setPosition(SERVO_POS_A);
            if (servo2 != null) servo2.setPosition(SERVO_POS_A);
        } else if (gamepad1.b) {
            // Press B to go to Position 2 (e.g., 0.0)
            if (servo1 != null) servo1.setPosition(SERVO_POS_B);
            if (servo2 != null) servo2.setPosition(SERVO_POS_B);
        }
        // Note: No "else" block needed. Standard servos hold their last position.
    }

    public void initFlywheelMotor(){
        try {
            flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
            flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            flywheel = null;
            telemetry.addData("Error", "Flywheel motor not found");
        }
    }

    public void initCollector() {
        try {
            collector = hardwareMap.get(DcMotorEx.class, "collector");
            collector.setDirection(DcMotorSimple.Direction.FORWARD);
            collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            collector = null;
            telemetry.addData("Error", "Collector motor not found");
        }
    }

    // Updated Init for Standard Servos
    public void initServos() {
        try {
            // Changed type to Servo.class
            servo1 = hardwareMap.get(Servo.class, "servoOne");
            servo2 = hardwareMap.get(Servo.class, "servoTwo");

            // Optional: Set startup positions
            // servo1.setPosition(SERVO_POS_B);
            // servo2.setPosition(SERVO_POS_B);

        } catch (Exception e) {
            servo1 = null;
            servo2 = null;
            telemetry.addData("Error", "Servos not found");
        }
    }

    public void collector(){
        if(gamepad1.right_trigger >= 0.25){
            if (collector != null) collector.setPower(1.0);
        }
        else if(gamepad1.left_trigger >= 0.25){
            if (collector != null) collector.setPower(-1.0);
        }
        else{
            if (collector!=null) collector.setPower(0.0);
        }
    }

    public void telemetryData(){
        // Flywheel Telemetry
        if (flywheel != null) {
            double ticksPerSecond = flywheel.getVelocity();
            double rpm = (ticksPerSecond / TICKS_PER_REV) * 60;
            telemetry.addData("Flywheel Power", "%.2f", currentFlywheelPower);
            telemetry.addData("Flywheel RPM", "%.2f", rpm);
        } else {
            telemetry.addData("Flywheel Status", "Not Initialized");
        }

        // Servo Telemetry (Now shows Position instead of Power)
        if (servo1 != null && servo2 != null) {
            telemetry.addData("Servo 1 Pos", "%.2f", servo1.getPosition());
            telemetry.addData("Servo 2 Pos", "%.2f", servo2.getPosition());
        } else {
            telemetry.addData("Servo Status", "Not Initialized");
        }
        telemetry.update();
    }
}