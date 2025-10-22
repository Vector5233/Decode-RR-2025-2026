package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**Configuration file
 * ControlHub
 * Motor port 00: motor0ne
 * Motor port 01: motorTwo
 * Motor port 02: motorThree
 */

@TeleOp(group = "Primary", name = "DC Motor Run to position")
@Disabled

public class Example_002b_DCMotorThree extends LinearOpMode {
    //Global Variables
    private DcMotor motorOne;
    double motorOneZeroPower = 0.0;
    double motorOnePower = 1.0;

    private DcMotor motorTwo;
    double motorTwoZeroPower = 0.0;
    double motorTwoSensitivity = 0.5;

    private DcMotor motorThree;
    double motorThreeZeroPower = 0.0;
    double motorThreePower = 1.0;
    int motorThreePositionOne = 0;
    int motorThreePositionTwo = 1000;


    @Override
    public void  runOpMode() throws InterruptedException{
        initHardware();
        while(!isStarted()){
            motorTelemetry();
        }
        waitForStart();
        while(opModeIsActive()){
            teleOpControls();
            motorTelemetry();
        }
    }
    public void initHardware(){
        initMotorOne();
        initMotorTwo();
        initMotorThree();
    }

    public  void  initMotorOne(){
        motorOne = hardwareMap.get(DcMotor.class, "motorOne");
        motorOne.setDirection(DcMotorSimple.Direction.FORWARD);
        motorOne.setPower(motorOneZeroPower);
        motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//FLOAT for cost to a stop
        motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
}
    public void  initMotorTwo(){
        motorTwo = hardwareMap.get(DcMotor.class, "motorTwo");
        motorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTwo.setPower(motorTwoZeroPower);
        motorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//FLOAT for cost to a stop
        motorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
          STOP_AND_RESET_ENCODER - The encoder's current position is set to zero
          RUN_WITHOUT_ENCODER - The encoder data is collected, but the velocity of the motor is set to a specific power level without the PID.
          RUN_USING_ENCODER - The generic PID contoller is used to achieved a specific power using encoder data
          RUN_TO_POSITION - Motor will attempt to run to a "target" encoder position
         */
    }

    public void initMotorThree(){
        motorThree = hardwareMap.get(DcMotor.class, "motorThree");
        motorThree.setDirection(DcMotorSimple.Direction.FORWARD);
        motorThree.setPower(motorThreeZeroPower);
        motorThree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //BRAKE stop as fast as possible
        motorThree.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorThree.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void  motorTelemetry(){
//        telemetry.addData("motorOne", "Encoder: %2d, Power:%.2f", motorOne.getCurrentPosition(), motorOne.getPower());
//        telemetry.update();
//        telemetry.addData("motorTwo", "Encoder: %2d, Power:%.2f", motorTwo.getCurrentPosition(), motorTwo.getPower());
        telemetry.addData("Note", "Tap y to reset Encoders");
        telemetry.addData("motorThree","Encoder: %2d, Power: %2f", motorThree.getCurrentPosition(), motorThree.getPower());
        telemetry.update();
    }
    public void runMotorThreeToPosition(int position) {
        motorThree.setTargetPosition(position);
        motorThree.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorThree.setPower(motorThreePower);
        while (motorThree.isBusy()) {
            motorTelemetry();
        }
        motorThree.setPower(motorThreeZeroPower);// Optional /
    }

    public void resetEncoders(){
        stopMotors();
        stopAndResetEncoders();
        resetMode();
    }
    public void stopMotors(){
        motorThree.setPower(motorThreeZeroPower);
    }
    public void stopAndResetEncoders(){
        motorThree.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void resetMode(){
        motorThree.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void teleOpControls() {
        //Button control motor
        if (gamepad2.x) {
            motorOne.setPower(-motorOnePower);
        }
        if (gamepad2.a) {
            motorOne.setPower(motorOneZeroPower);
        }
        if (gamepad2.b) {
            motorOne.setPower(motorOnePower);
        }
        // run motor with joyStick
        motorTwo.setPower(gamepad2.right_stick_y * motorTwoSensitivity);

        if (gamepad2.left_bumper) {
            runMotorThreeToPosition(motorThreePositionOne);
        }
        if (gamepad2.right_bumper) {
            runMotorThreeToPosition(motorThreePositionTwo);
        }
        if (gamepad2.y) {
            resetEncoders();

        }
    }


}
