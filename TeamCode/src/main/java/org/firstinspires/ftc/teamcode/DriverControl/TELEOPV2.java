package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**Configuration file
 * Drive train motors will called - we many want to make a diveMotor class later - Chad this would be your input
 * motor names in the configuration:  on Robot Controller: this is where you name the motors in the app
 * frontLeftMotor
 * frontRightMotor
 * backLeftMotor
 * backRightMotor
 * flyWheel
 * collector
 * leftLift
 * rightLift
 * flpperOne
 * flipperTwo
 */
@Disabled
@TeleOp(group = "Primary", name = "Basic TeleOp Structure")
public class TELEOPV2 extends LinearOpMode {
    //declare motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //collector and shooter motors can be declared here when needed
    private DcMotor flyWheel;
    private DcMotor collector;

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

    public void initDriveTrain(){



    }

    public void teleoOpControls(){
        //mechanum drive code here for controls




}
