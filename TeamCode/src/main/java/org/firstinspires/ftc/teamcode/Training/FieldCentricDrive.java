package org.firstinspires.ftc.teamcode.Training;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

/**
 * Configuration file
 * Motor port 1 : leftFM
 * Motor port 2 : rightFM
 * Motor port 3 : leftBM
 * Motor port 4 : rightBM
 */

@Disabled
@TeleOp(group = "Primary", name = "Short Name")
public class FieldCentricDrive extends LinearOpMode {
    private DcMotor leftfront_drive;
    private DcMotor rightfront_drive;
    private DcMotor leftback_drive;
    private DcMotor rightback_drive;
    private IMU pinpoint;
    double lx = gamepad1.left_stick_x;
    double ly = gamepad1.left_stick_y;
    double rx = gamepad1.right_stick_x;

    Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
        }

        initIMU();

        waitForStart();


        while (opModeIsActive()) {

            teleOpControls();


        }
    }

    public void initHardware() {
        initIMU();
        initDriveMotors();

    }

    public void initDriveMotors() {
        leftfront_drive = hardwareMap.get(DcMotor.class, "leftfront_drive");
        rightfront_drive = hardwareMap.get(DcMotor.class, "rightfront_drive");
        leftback_drive = hardwareMap.get(DcMotor.class, "leftback_drive");
        rightback_drive = hardwareMap.get(DcMotor.class, "rightback_drive");

        // set oneside to reverse
        leftfront_drive.setDirection(DcMotor.Direction.REVERSE);
        leftback_drive.setDirection(DcMotor.Direction.REVERSE);


    }

    public void initIMU() {
        pinpoint = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        pinpoint.initialize(parameters);


    }

    public void teleOpControls() {
        double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);
        double power = 0.8 + (0.6 * gamepad1.right_trigger);

        if (gamepadRateLimit.hasExpired() && gamepad1.a) {
            pinpoint.resetYaw();
            gamepadRateLimit.reset();
        }

        double heading = -pinpoint.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
        double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);

        leftfront_drive.setPower(((adjustedLy + adjustedLx + rx) / max) * power);
        leftback_drive.setPower(((adjustedLy - adjustedLx + rx) / max) * power);
        rightfront_drive.setPower(((adjustedLy - adjustedLx - rx) / max) * power);
        rightback_drive.setPower(((adjustedLy + adjustedLx - rx) / max) * power);
    }
}
