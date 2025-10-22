package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**Configuration file
 *
 */

@Disabled
@TeleOp(group = "Primary", name = "Text Output")
public class Example_002a_HelloWorld extends LinearOpMode {
    @Override
    public void  runOpMode() throws InterruptedException{
        initHardware();
        while(!isStarted()){
            telemetry.addData("Hello", " World 1");
            telemetry.update();

        }
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Hello", " World 2");
            telemetry.update();

        }
    }
    public void initHardware(){

    }
}
