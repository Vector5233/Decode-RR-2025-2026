package org.firstinspires.ftc.teamcode.Tutorial;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@Disabled
@TeleOp (group = "Tutorial", name = "BASIC TELOP KF")
public class BASICTELOPKF extends LinearOpMode {

    private VoltageSensor batteryVoltageSensor;

    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
            //code to run while waiting for start
            //Add  telemetry here if needed
            telemetryVoltage();

        }
        waitForStart();
        while (opModeIsActive()) {
            //code to run while op mode is active
            //Add telemetry here if needed
            telemetryVoltage();

        }

    }

    public void initHardware() {
        //code to initialize hardware
        initVoltageSensor();
    }

    public void initVoltageSensor() {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void telemetryVoltage() {
        double voltage = batteryVoltageSensor.getVoltage();
        telemetry.addData("Battery Voltage ", voltage);
        telemetry.update();
    }
}
