package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


//@Disabled
@TeleOp (group = "DriverControl", name = "Teleop")
public class TELOPV1 extends LinearOpMode {
    // Declare all hardware variables here private or public as needed
    private DcMotor flywheelMotor;
    private DcMotor noodleMotor;

    private double targetFlywheelPower = 0.0;
    private double currentFlywheelPower = 0.0;
    private final double FLYWHEEL_RAMP = 0.02; // step per loop - tune as needed
    private final double FLYWHEEL_ON_POWER = 0.85; // initial guess; tune

    private double noodlePower = 0.0;
    private boolean lastRightBumper = false;


    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
            //code to run while waiting for start
            //Add  telemetry here if needed


        }
        waitForStart();
        while (opModeIsActive()) {
            //code to run while op mode is active
            //Add telemetry here if needed
            boolean currRightBumper = gamepad1.right_bumper;
            if (currRightBumper && !gamepad1.right_bumper) {
                if (Math.abs((targetFlywheelPower)) < 0.01) {
                    targetFlywheelPower = FLYWHEEL_ON_POWER;
                } else {
                    targetFlywheelPower = 0.0;
                }
            }
            lastRightBumper = currRightBumper;

            if (Math.abs(currentFlywheelPower - targetFlywheelPower) > FLYWHEEL_RAMP) {
                if (currentFlywheelPower < targetFlywheelPower) {
                    currentFlywheelPower += FLYWHEEL_RAMP;
                } else {
                    currentFlywheelPower -= FLYWHEEL_RAMP;
                }
            } else {
                currentFlywheelPower = targetFlywheelPower;
            }

            if (flywheelMotor != null) {
                flywheelMotor.setPower(currentFlywheelPower);
            }
        }
        boolean currLeftBumper = gamepad1.left_bumper;
        if (currLeftBumper && !gamepad1.left_bumper) {
            if (Math.abs(noodlePower) < 0.01) {
                noodlePower = 1.0;
            }
        }
    }

    public void initHardware() {
        //code to initialize hardware
        initFlywheelMotor();
        initNoodleMotor();
    }

    public void initFlywheelMotor() {
       try {
           flywheelMotor = hardwareMap.get(DcMotor.class, "flywheelMotor");
       }
         catch (Exception e) {
             flywheelMotor = null;
         }
    }
    public void initNoodleMotor() {
        noodleMotor = hardwareMap.get(DcMotor.class, "noodleMotor");
        noodleMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        noodleMotor.setPower(noodlePower);
        noodleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        noodleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//FLOAT for coast to a stop

    }

}
