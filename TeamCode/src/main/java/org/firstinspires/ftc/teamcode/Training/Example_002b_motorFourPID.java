package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**Configuration file
 * ControlHub
 * Motor port 00: motor0ne
 * Motor port 01: motorTwo
 * Motor port 02: motorThree
 * Motor port 03: motorFour
 */

@Disabled
@TeleOp(group = "Primary", name = "DC Motor Run to position")
public class Example_002b_motorFourPID extends LinearOpMode {
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

    private DcMotorEx motorFour;
    double motorFourZeroPower = 0.0;
    double motorFourMaxPower = 1.0;
    double motorFourCurrentVelocity = 0.0;
    double motorFourMaxVelocity = 0.00;

    /**
     * Build and Run Example Max Velocity Test to find target value
     * Recommendation: Target velocity should be no more than 80% of the max velocity
     */

    // PID Variables
    double motorFourTargetVelocity = 800.0;
    double resultMaxVelocityTest = 2800.0; // Enter max Velocity from Test
    double F = 32767.0/ resultMaxVelocityTest; // F for feedforward
    double kP = F * 0.1; // Discuss P term oscillation. = proportional
    double kI = kP * 0.1; // I term nudges you to your target over time.
    double kD = kP * 0.01; // D term applies a breaking force to control over shoot. Keep this term small to avoid noise
    double position = 5.0;

    /**Tuning Guide
     * https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
     * Run the opMode and get the kP value from the telemetry, then make adjustments using the following video.
     * https://www.youtube.com/watch?v=6OH-wOsVVjg
     */


    @Override
    public void  runOpMode() throws InterruptedException{
        initHardware();
        while(!isStarted()){
            motorTelemetry();
        }
        waitForStart();
        while(opModeIsActive()){
//            teleOpControls();
//            motorFourMaxVelocityTest();
            runMotorFour(motorFourTargetVelocity);
            motorTelemetry();
        }
    }
    public void initHardware(){
        initMotorOne();
        initMotorTwo();
        initMotorThree();
        initMotorFour(kP, kI, kD, F, position);
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

    public void initMotorFour(double kP, double kI, double kD, double F, double position){
        motorFour=hardwareMap.get(DcMotorEx.class,"motorFour");
        motorFour.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFour.setPower(motorFourZeroPower);
        motorFour.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFour.setVelocityPIDFCoefficients(kP,kI,kD,F);
        motorFour.setPositionPIDFCoefficients(position);
        motorFour.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFour.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }
    public void  motorTelemetry(){
//        telemetry.addData("motorOne", "Encoder: %2d, Power:%.2f", motorOne.getCurrentPosition(), motorOne.getPower());
//        telemetry.update();
//        telemetry.addData("motorTwo", "Encoder: %2d, Power:%.2f", motorTwo.getCurrentPosition(), motorTwo.getPower());
//        telemetry.addData("Note", "Tap y to reset Encoders");
//        telemetry.addData("motorThree","Encoder: %2d, Power: %2f", motorThree.getCurrentPosition(), motorThree.getPower());
//        telemetry.addData("Current Power", motorFour.getPower());
//        telemetry.addData("Maximum Velocity", motorFourMaxVelocity);
//        telemetry.addData("Current Velocity", motorFourCurrentVelocity);
        telemetry.addData("Power", motorFour.getPower());
        telemetry.addData("Target Velocity", motorFourTargetVelocity);
        telemetry.addData("Current Velocity", motorFour.getVelocity());
        telemetry.addData("Maximum Velocity", motorFourMaxVelocity);
        telemetry.addData("F", F);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.update();
    }
    public void runMotorThreeToPosition(int position) {
        motorThree.setTargetPosition(position);
        motorThree.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorThree.setPower(motorThreePower);
        while (motorThree.isBusy()) {
            motorTelemetry();
        }
//        motorThree.setPower(motorThreeZeroPower);// Optional /
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

    public void motorFourMaxVelocityTest(){
        motorFour.setPower(motorFourMaxPower);
        motorFourCurrentVelocity = motorFour.getVelocity();
        if(motorFourCurrentVelocity > motorFourMaxVelocity){
            motorFourMaxVelocity = motorFourCurrentVelocity;
        }
    }

    public void runMotorFour(double velocity){
        motorFour.setVelocity(velocity);
        motorFourCurrentVelocity = motorFour.getVelocity();
        if(motorFourCurrentVelocity > motorFourMaxVelocity){
            motorFourMaxVelocity = motorFourCurrentVelocity;
        }
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
