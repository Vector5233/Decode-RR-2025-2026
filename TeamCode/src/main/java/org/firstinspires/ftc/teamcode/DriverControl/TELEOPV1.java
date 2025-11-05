package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo; // Import the Servo class
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp (group = "DriverControl", name = "Teleop")
public class TELEOPV1 extends LinearOpMode {

    private DcMotorEx flywheelMotor;
    private final double TICKS_PER_REV = 28; // Example for a

    // Declare Continuous Servo variables
    private Servo continuousServo1;
    private Servo continuousServo2;

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

            // --- Continuous Servo Logic ---
            if (gamepad1.a) {
                // When 'A' is pressed, run servos at full power
                if(continuousServo1 != null) continuousServo1.setPower(1.0);
                if(continuousServo2 != null) continuousServo2.setPower(1.0);
            } else {
                // When 'A' is not pressed, stop the servos
                // For continuous rotation servos, 0.5 is the standard stop value.
                if(continuousServo1 != null) continuousServo1.setPower(0.5);
                if(continuousServo2 != null) continuousServo2.setPower(0.5);
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
    }

    // New method to initialize the continuous servos
    public void initContinuousServos() {
        try {
            continuousServo1 = hardwareMap.get(Servo.class, "LF");
            continuousServo2 = hardwareMap.get(Servo.class, "RF");
        } catch (Exception e) {
            continuousServo1 = null;
            continuousServo2 = null;
            telemetry.addData("Error", "One or both servos not found");
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

        // Servo Telemetry
        if (continuousServo1 != null && continuousServo2 != null) {
            telemetry.addData("Servo 1 Power", "%.2f", continuousServo1.getPower());
            telemetry.addData("Servo 2 Power", "%.2f", continuousServo2.getPower());
        } else {
            telemetry.addData("Servo Status", "Not Initialized");
        }
        telemetry.update();
    }
}
