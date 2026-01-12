package org.firstinspires.ftc.teamcode.DriverControl;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(group = "Primary", name = "Block 'Em Sock 'Em AUTO")
public class AUTO extends LinearOpMode {

    // --- Hardware (Only Attachments) ---
    // Note: Drive motors are handled by 'MecanumDrive', so we don't declare them here.
    private DcMotorEx flywheel;
    private DcMotor collector;
    private DcMotorEx leftLift, rightLift;
    private Servo servo1, servo2;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Initialize the Drive Train (RoadRunner)
        // Make sure your MecanumDrive.java class uses the correct motor names!
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // 2. Initialize Attachments (Lift, Intake, etc.)
        initAttachments();

        // 3. Define the Path (Build it BEFORE start)
        Action trajectory = drive.actionBuilder(beginPose)
                .lineToX(24) // Drive forward 24 inches
                .build();

        // 4. Wait for Start
        while (!isStarted()) {
            telemetry.addData("Status", "Ready for Auto");
            telemetry.addData("Path", "Forward 24 Inches");
            telemetry.update();
        }

        waitForStart();

        // 5. Run the Path ONCE
        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            trajectory
                            // You can add lift/servo actions here later
                    )
            );
        }
    }

    public void initAttachments() {
        // --- Lift Motors ---
        try {
            leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
            rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

            leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
            rightLift.setDirection(DcMotorSimple.Direction.FORWARD);

            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addData("Warning", "Lifts not found");
        }

        // --- Intake & Flywheel ---
        try {
            flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
            collector = hardwareMap.get(DcMotor.class, "collector");
        } catch (Exception e) {
            telemetry.addData("Warning", "Intake/Flywheel not found");
        }

        // --- Servos ---
        try {
            servo1 = hardwareMap.get(Servo.class, "servoOne");
            servo2 = hardwareMap.get(Servo.class, "servoTwo");
            servo2.setDirection(Servo.Direction.REVERSE);

            // Lock servos on init
            servo1.setPosition(0.5);
            servo2.setPosition(0.5);
        } catch (Exception e) {
            telemetry.addData("Warning", "Servos not found");
        }
    }
}