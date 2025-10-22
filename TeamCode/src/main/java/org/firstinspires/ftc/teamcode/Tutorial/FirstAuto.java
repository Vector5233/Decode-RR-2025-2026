package org.firstinspires.ftc.teamcode.Tutorial;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name="FirstAuto")
public class FirstAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // creating starting pose:
        Pose2d beginPose = new Pose2d(new Vector2d(0, 63), Math.toRadians(270));

        // creating RR Drive object
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Action path = drive.actionBuilder(beginPose)
                .splineToSplineHeading(new Pose2d(40,0,Math.toRadians(180)), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(40,-48), Math.toRadians(0))
                .lineToXConstantHeading(0)
                .build();

        

                Actions.runBlocking(new SequentialAction(path));

    }
}
