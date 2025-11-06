package org.firstinspires.ftc.teamcode.Tutorial;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Disabled
@TeleOp (group = "Tutorial", name = "BASIC TELEOP RD")
public class BASICTELEOPRD extends LinearOpMode {

    private VoltageSensor batteryVoltageSensor;
    DcMotor frontLeftM;
    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();
        while(!isStarted()){
            telemetryVoltage();
        }
        waitForStart();
        while(opModeIsActive()){
            telemetryVoltage();
        }
    }



public void initHardware(){
    initVoltageSensor();
}

public void initVoltageSensor(){
    batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
}

public void telemetryVoltage(){
        double voltage = batteryVoltageSensor.getVoltage();
        telemetry.addData("Battery Voltage ", voltage);
        telemetry.update();
}

}