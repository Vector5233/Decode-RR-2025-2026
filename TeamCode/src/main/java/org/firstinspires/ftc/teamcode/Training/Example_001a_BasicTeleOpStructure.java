package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**Configuration file
 *
 */

@Disabled
@TeleOp(group = "Primary", name = "Short Name")
public class Example_001a_BasicTeleOpStructure extends LinearOpMode {
    @Override
    public void  runOpMode() throws InterruptedException{
        initHardware();
        while(!isStarted()){}
        waitForStart();
        while(opModeIsActive()){}
    }
    public void initHardware(){}
}
