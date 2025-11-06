package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp (group = "DriverControl", name = "Teleop")
public class TELEOPV1 extends LinearOpMode {

    private DcMotorEx flywheelMotor;
    // TODO: Find and set the correct ticks per revolution for your motor
    // Common values are 28, 145.1, 384.5, 537.7 for different FTC motors
    private final double TICKS_PER_REV = 28; // Example for a

    private double targetFlywheelPower = 0.0;
    private double currentFlywheelPower = 0.0;
    private final double FLYWHEEL_RAMP = 0.02; // step per loop, probably needs changing
    private final double FLYWHEEL_ON_POWER = 1.00  ; // initial guess, definitely needs changing

    private boolean lastRightBumper = false;

    public void runOpMode() throws InterruptedException{
        initHardware();
        while (!isStarted()){

        }
        waitForStart();
        while (opModeIsActive()){
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
        }
    }



public void initHardware(){
        initFlywheelMotor();
}

public void initFlywheelMotor(){
    try {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    } catch (Exception e) {
        flywheelMotor = null;
        telemetry.addData("Error", "Flywheel motor not found");
        telemetry.update();
    }

    }
    public void telemetryData(){
        if (flywheelMotor != null) {
            // Get velocity in ticks per second from the encoder
            double ticksPerSecond = flywheelMotor.getVelocity();
            // Convert ticks per second to revolutions per minute (RPM)
            double rpm = (ticksPerSecond / TICKS_PER_REV) * 60;

            telemetry.addData("Flywheel Power", "%.2f", currentFlywheelPower);
            telemetry.addData("Flywheel RPM", "%.2f", rpm);
        } else {
            telemetry.addData("Flywheel Status", "Not Initialized");
        }
        telemetry.update();

    }
}