package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**Configuration file
 * Control Hub:
 * Servo Port 00: servoOne // must match variable name in CH and in Java
 * Servo Port 01: servoTwo
 */

@Disabled
@TeleOp(group = "Primary", name = "Servo with buttons")
public class Example_001d_ServoTwo extends LinearOpMode {

    //global variables go below the class name
    private Servo servoOne; // servos go from 0 to 1 rotates 180 degrees
    double servoOneInitPosition =0.5; // doubles store a decimal
    double servoOnePositionOne =0.0;
    double servoOnePositionTwo =1.0;
    int servoOneDelay = 10; //Try not to use sleep/delay in Teleop

    private Servo servoTwo;
    double servoTwoInitPosition =0.50;
    double servoTwoSensitivity =1.0;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        while(!isStarted()){
            servoTelemetry();

        }
        waitForStart();
        while(opModeIsActive()){
            servoTelemetry();
            teleOpControls();


        }

    }

    public void initHardware(){
        initServoOne(); // call to the method other methods/ hardware pieces - method stacking
        initServoTwo();
    }
    public void servoTelemetry(){
        //telemetry.log().clear(); don't use
        telemetry.addData("Position", servoOne.getPosition());
        telemetry.addData("Position", servoTwo.getPosition());
        telemetry.addData("Direction", servoOne.getDirection());
        telemetry.addData("Direction", servoTwo.getDirection());
//        telemetry.addData("Controller", servoOne.getController());
//        telemetry.addData("Port Number", servoOne.getConnectionInfo());
//        telemetry.addData("Device Name", servoOne.getDeviceName());
//        telemetry.addData("Manufacture", servoOne.getManufacturer());
//        telemetry.addData("Version", servoOne.getVersion());
//        telemetry.addData("Class", servoOne.getClass());
        telemetry.update();
    }
    public void initServoOne(){
        servoOne = hardwareMap.get(Servo.class, "servoOne"); // maps the servo
        servoOne.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        servoOne.setPosition(servoOneInitPosition); // sets the initial position from the variable above.
    }
    public  void initServoTwo(){
        servoTwo=hardwareMap.get(Servo.class, "servoTwo");
        servoTwo.setDirection(Servo.Direction.REVERSE);
        servoTwo.setPosition(servoTwoInitPosition);
    }
    public void  teleOpControls(){
        if(gamepad1.a){
            servoOne.setPosition(servoOnePositionOne);
        }
        if (gamepad1.b){
            servoOne.setPosition(servoOnePositionTwo);
        }
        if(gamepad1.right_bumper){
            servoOneSlower(servoOnePositionOne, servoOnePositionTwo, servoOneDelay);
        }
        servoTwo.setPosition(servoTwoSensitivity - (gamepad1.right_stick_y * servoTwoSensitivity));

    }
    public void servoOneSlower(double startPosition, double endPosition, int delay){
        // maybe for Auto mode.
        double range =((endPosition - startPosition) * 100);
        //for(local variable; conditional; update variable)
        for(int i =0; i<= range; i++){
            servoOne.setPosition(startPosition);
            sleep(delay);
            startPosition = startPosition + 0.01;
        }
    }
}
