package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**Configuration file
 *
 */

@Disabled
@TeleOp(group = "DriverControlled", name = "Basic TeleOp Structure")
public class Example_001a_BasicTeleOpStructure extends LinearOpMode {
    @Override
    public void  runOpMode() throws InterruptedException{
        // Call to method to initialize hardware
        initHardware();
        // Wait for the start button to be pressed
        // Loop to provide telemetry before start
        while(!isStarted()){
            // Telemetry goes here
        }
        waitForStart();
        while(opModeIsActive()){
            // TeleOp controls and telemetry go here
        }
    }
    public void initHardware(){
        // Initialize hardware pieces here
    }
}
