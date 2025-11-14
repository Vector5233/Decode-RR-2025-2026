package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (group = "DriverControl", name = "Teleop")
public class TELEOPV1 extends LinearOpMode {

    private DcMotorEx flywheelMotor;
    private DcMotorEx flywheelMotor2; // **ADDED: Second motor for the flywheel**
    // TODO: Find and set the correct ticks per revolution for your motor
    // Common values are 28, 145.1, 384.5, 537.7 for different FTC motors
    private final double TICKS_PER_REV = 28; // Example for a

    // Declare Continuous Servo variables
    // **FIXED: Changed type from Servo to CRServo**
    private CRServo continuousServo1;
    private CRServo continuousServo2;

    private double targetFlywheelPower = 0.0;
    private double currentFlywheelPower = 0.0;
    private final double FLYWHEEL_RAMP = 0.02; // step per loop, probably needs changing
    private final double FLYWHEEL_ON_POWER = 1.00  ; // initial guess, definitely needs changing

    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();
        while (!isStarted()){
            // You can place initialization telemetry here
            telemetry.update(); // Add update to see init messages
        }
        waitForStart();
        while (opModeIsActive()){
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

            if (flywheelMotor != null) {
                flywheelMotor.setPower(currentFlywheelPower);
            }
            if (flywheelMotor2 != null) {
                flywheelMotor2.setPower(currentFlywheelPower); // Use the same power variable
            }

            // --- Continuous Servo Logic (e.g., Intake) ---
            if (gamepad1.a) {
                // When 'A' is pressed, run servos forward (e.g., intake)
                if(continuousServo1 != null) continuousServo1.setPower(1.0);
                if(continuousServo2 != null) continuousServo2.setPower(1.0);
            } else if (gamepad1.b) {
                // **Added: Use 'B' to reverse the servos (e.g., outtake/reverse)**
                if(continuousServo1 != null) continuousServo1.setPower(-1.0);
                if(continuousServo2 != null) continuousServo2.setPower(-1.0);
            } else {
                // When neither is pressed, stop the servos.
                // For continuous rotation servos, a power of 0.0 (or setPower(0)) is stop.
                // 0.5 is the stop value for a standard Servo *command* to a CRServo, but setPower(0.0) is clearer and standard.
                if(continuousServo1 != null) continuousServo1.setPower(0.0); // **FIXED: Changed stop power to 0.0**
                if(continuousServo2 != null) continuousServo2.setPower(0.0); // **FIXED: Changed stop power to 0.0**
            }

            // Call the telemetry update method
            telemetryData();
        }
    }

    public void initHardware(){
        initFlywheelMotor();
        initContinuousServos(); // Call the new servo initialization method
    }

    public void initFlywheelMotor(){
        try {
            flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
            flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            flywheelMotor = null;
            telemetry.addData("Error", "Flywheel motor not found");
        }
        // --- ADD THIS BLOCK for the second motor ---
        try {
            flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");
            // IMPORTANT: The second motor often needs to be the opposite direction
            flywheelMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
            flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            flywheelMotor2 = null;
            telemetry.addData("Error", "Flywheel motor 2 (flywheelMotor2) not found");
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

    public void telemetryData(){
        // Flywheel Telemetry
        if (flywheelMotor != null) {
            double ticksPerSecond = flywheelMotor.getVelocity();
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
