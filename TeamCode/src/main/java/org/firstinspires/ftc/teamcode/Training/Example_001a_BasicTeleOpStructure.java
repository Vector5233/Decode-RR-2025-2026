package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**Configuration file
 *
 */

@Disabled
@TeleOp(group = "Primary", name = "Basic TeleOp Structure")
public class Example_001a_BasicTeleOpStructure extends LinearOpMode {
    @Override
    public void  runOpMode() throws InterruptedException{
       //method to initialize hardware
        initHardware();
        while(!isStarted()){

            //code to run while waiting for start
            //Add telemetry here if needed

        }
        waitForStart();
        while(opModeIsActive()){
            //code to run while op mode is active
            //Add telemetry here if needed
        }
    }
    public void initHardware(){
        //code to initialize hardware
    }
}
