package org.firstinspires.ftc.teamcode.Training;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private DcMotor leftFM;
    private DcMotor rightFM;
    private DcMotor leftBM;
    private DcMotor rightBM;
    private IMU imu;
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
        leftFM = hardwareMap.get(DcMotor.class, "leftFM");
        rightFM = hardwareMap.get(DcMotor.class, "rightFM");
        leftBM = hardwareMap.get(DcMotor.class, "leftBM");
        rightBM = hardwareMap.get(DcMotor.class, "rightBM");

        // set oneside to reverse
        leftFM.setDirection(DcMotor.Direction.REVERSE);
        leftBM.setDirection(DcMotor.Direction.REVERSE);


    }

    public void initIMU() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);


    }

    public void teleOpControls() {
        double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);
        double power = 0.8 + (0.6 * gamepad1.right_trigger);

        if (gamepadRateLimit.hasExpired() && gamepad1.a) {
            imu.resetYaw();
            gamepadRateLimit.reset();
        }

        double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
        double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);

        leftFM.setPower(((adjustedLy + adjustedLx + rx) / max) * power);
        leftBM.setPower(((adjustedLy - adjustedLx + rx) / max) * power);
        rightFM.setPower(((adjustedLy - adjustedLx - rx) / max) * power);
        rightBM.setPower(((adjustedLy + adjustedLx - rx) / max) * power);
    }
}
