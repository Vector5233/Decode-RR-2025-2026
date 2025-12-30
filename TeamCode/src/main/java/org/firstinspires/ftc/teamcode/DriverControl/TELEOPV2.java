package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
/**Configuration file
 * Drive train motors will called - we many want to make a diveMotor class later - Chad this would be your input
 * motor names in the configuration:  on Robot Controller: this is where you name the motors in the app
 * frontLeftMotor
 * frontRightMotor
 * backLeftMotor
 * backRightMotor
 */
@Disabled
@TeleOp(group = "Primary", name = "Basic TeleOp Structure")
public class TELEOPV2 extends LinearOpMode {
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
