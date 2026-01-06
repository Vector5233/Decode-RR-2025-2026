package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**Configuration file
 * Drive train motors will called - we many want to make a diveMotor class later - Chad this would be your input
 * motor names in the configuration:  on Robot Controller: this is where you name the motors in the app
 * frontLeftMotor
 * frontRightMotor
 * backLeftMotor
 * backRightMotor
 * flyWheel
 * collector
 * leftLift
 * rightLift
 * flpperOne
 * flipperTwo
 */

@TeleOp(group = "Primary", name = "Pick This One")
public class TELEOPV2 extends LinearOpMode {
    //declare motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotorEx flywheel;
    private DcMotorEx flywheelMotor2; // **ADDED: Second motor for the flywheel**
    // TODO: Find and set the correct ticks per revolution for motor
    // Common values are 28, 145.1, 384.5, 537.7 for different FTC motors
    private final double TICKS_PER_REV = 28; // Example for a
    // Declare Continuous Servo variables
    private CRServo continuousServo1;
    private CRServo continuousServo2;
    private double targetFlywheelPower = 0.0;
    private double currentFlywheelPower = 0.0;
    private final double FLYWHEEL_RAMP = 0.02; // step per loop, probably needs changing
    private final double FLYWHEEL_ON_POWER = 1.00; // initial guess, definitely needs changing
    private boolean lastRightBumper = false;


    //collector and shooter motors can be declared here when needed
    private DcMotor collector;

    @Override
    public void runOpMode() throws InterruptedException {
        //method to initialize hardware
        initHardware();
        while (!isStarted()) {
            telemetryData();
            //code to run while waiting for start
            //Add telemetry here if needed

        }
        waitForStart();
        while (opModeIsActive()) {
            //code to run while op mode is active
            //Add telemetry here if needed
            driveTrain();
            flywheels();
            telemetryData();
            collector();
        }
    }

    public void initHardware() {
        initDriveTrain();
        initFlywheelMotor();
        initContinuousServos();
        initCollector();
    }

    public void initDriveTrain() {

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
    }

    public void driveTrain() {
        //mechanum drive code here for controls
        double modifier = gamepad1.x ? 0.5 : 1.0;

        double drive = -gamepad1.left_stick_y;  // Forward/backward (negated)
        double strafe = gamepad1.left_stick_x;  // Left/right
        double turn = gamepad1.right_stick_x; // Rotation

        double frontLeftPower = drive + strafe + turn;
        double backLeftPower = drive - strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backRightPower = drive + strafe - turn;

        // Put the powers in an array to find the maximum
        double[] powers = {frontLeftPower, backLeftPower, frontRightPower, backRightPower};
        double max = 1.0;

        for (double p : powers) {
            if (Math.abs(p) > max) {
                max = Math.abs(p);
            }
        }
                // Divide all by the max to stay within -1 to 1
        frontLeft.setPower(frontLeftPower / max * modifier);
        backLeft.setPower(backLeftPower / max * modifier);
        frontRight.setPower(frontRightPower / max * modifier);
        backRight.setPower(backRightPower / max * modifier);
    }

    public void flywheels() {
        // --- Flywheel Logic ---
        boolean currRightBumper = gamepad1.right_bumper;
        if (currRightBumper && !lastRightBumper) {
            if (Math.abs(targetFlywheelPower) < 0.01) {
                targetFlywheelPower = FLYWHEEL_ON_POWER;
            } else {
                targetFlywheelPower = 0.0;
            }
        }
        lastRightBumper = currRightBumper;
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
        if (gamepad1.a) {
            // When 'A' is pressed, run servos forward (e.g., intake)
            if (continuousServo1 != null) continuousServo1.setPower(1.0);
            if (continuousServo2 != null) continuousServo2.setPower(1.0);
        } else if (gamepad1.b) {
            // **Added: Use 'B' to reverse the servos (e.g., outtake/reverse)**
            if (continuousServo1 != null) continuousServo1.setPower(-1.0);
            if (continuousServo2 != null) continuousServo2.setPower(-1.0);
        } else {
            // When neither is pressed, stop the servos.
            // For continuous rotation servos, a power of 0.0 (or setPower(0)) is stop.
            // 0.5 is the stop value for a standard Servo *command* to a CRServo, but setPower(0.0) is clearer and standard.
            if (continuousServo1 != null)
                continuousServo1.setPower(0.0); // **FIXED: Changed stop power to 0.0**
            if (continuousServo2 != null)
                continuousServo2.setPower(0.0); // **FIXED: Changed stop power to 0.0**
        }
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
        // --- ADD THIS BLOCK for the second motor ---
    }
    public void initCollector() {
        try {
            collector = hardwareMap.get(DcMotorEx.class, "collector");
            collector.setDirection(DcMotorSimple.Direction.FORWARD);
            collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            collector = null;
            telemetry.addData("Error", "collector (collector) not found");
        }
    }
    // New method to initialize the continuous servos
    public void initContinuousServos() {
        try {
            // **FIXED: Changed hardware map type to CRServo.class**
            // **NOTE: Assuming "intake1" and "intake2" are the correct hardware map names.**
            // If the hardware map names "LFlywheel" and "RFlywheel" were correct, use those instead.
            continuousServo1 = hardwareMap.get(CRServo.class, "intake1");
            continuousServo2 = hardwareMap.get(CRServo.class, "intake2");
            // NOTE: You can set the direction of a CRServo if needed, e.g., continuousServo2.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            continuousServo1 = null;
            continuousServo2 = null;
            telemetry.addData("Error", "One or both continuous rotation servos not found");
        }
    }
    public void collector(){
        if(gamepad1.right_trigger >= 0){
            if (collector != null) collector.setPower(1.0);
        }
        else if(gamepad1.left_trigger >= 0){
            if (collector != null) collector.setPower(-1.0);
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
        if (flywheelMotor2 != null) {
            double ticksPerSecond2 = flywheelMotor2.getVelocity();
            double rpm2 = (ticksPerSecond2 / TICKS_PER_REV) * 60;
            telemetry.addData("Flywheel 2 Power", "%.2f", currentFlywheelPower);
            telemetry.addData("Flywheel 2 RPM", "%.2f", rpm2);
        } else {
            telemetry.addData("Flywheel 2 Status", "Not Initialized");
        }

        // Servo Telemetry
        if (continuousServo1 != null && continuousServo2 != null) {
            // CRServo uses .getPower() just like DcMotors/DcMotorEx
            telemetry.addData("Intake 1 Power", "%.2f", continuousServo1.getPower());
            telemetry.addData("Intake 2 Power", "%.2f", continuousServo2.getPower());
        } else {
            telemetry.addData("Intake Status", "Not Initialized");
        }
        telemetry.update();
    }
}
      ////    ////////////////////     ////
     ////    ////////////////////      ////