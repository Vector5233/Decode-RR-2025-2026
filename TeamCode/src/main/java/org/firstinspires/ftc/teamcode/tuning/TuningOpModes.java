package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.AngularRampLogger;
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.EncoderGroup;
import com.acmerobotics.roadrunner.ftc.EncoderRef;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.LateralPushTest;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.PinpointEncoderGroup;
import com.acmerobotics.roadrunner.ftc.PinpointIMU;
import com.acmerobotics.roadrunner.ftc.PinpointView;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class TuningOpModes {
    // Change to TankDrive.class if you ever switch to tank drive
    public static final Class<?> DRIVE_CLASS = MecanumDrive.class;

    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = false;

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    /**
     * Builds a PinpointView adapter so the RoadRunner tuning tools can read
     * encoder positions and heading velocity directly from the Pinpoint CPU.
     * Also allows the tuning tools to flip encoder directions interactively.
     */
    private static PinpointView makePinpointView(PinpointLocalizer pl) {
        return new PinpointView() {
            // Local copies of directions so we can toggle them without re-constructing
            // the localizer.  Initialized from the localizer's current settings.
            com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection parDir  = pl.parDirection;
            com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection perpDir = pl.perpDirection;

            @Override
            public void update() {
                pl.driver.update();
            }

            @Override
            public int getParEncoderPosition() {
                return pl.driver.getEncoderX();
            }

            @Override
            public int getPerpEncoderPosition() {
                return pl.driver.getEncoderY();
            }

            @Override
            public float getHeadingVelocity(UnnormalizedAngleUnit unit) {
                return (float) pl.driver.getHeadingVelocity(unit);
            }

            @Override
            public void setParDirection(@NonNull DcMotorSimple.Direction direction) {
                parDir = direction == DcMotorSimple.Direction.FORWARD
                        ? com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD
                        : com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED;
                pl.driver.setEncoderDirections(parDir, perpDir);
            }

            @Override
            public DcMotorSimple.Direction getParDirection() {
                return parDir == com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD
                        ? DcMotorSimple.Direction.FORWARD
                        : DcMotorSimple.Direction.REVERSE;
            }

            @Override
            public void setPerpDirection(@NonNull DcMotorSimple.Direction direction) {
                perpDir = direction == DcMotorSimple.Direction.FORWARD
                        ? com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD
                        : com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED;
                pl.driver.setEncoderDirections(parDir, perpDir);
            }

            @Override
            public DcMotorSimple.Direction getPerpDirection() {
                return perpDir == com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD
                        ? DcMotorSimple.Direction.FORWARD
                        : DcMotorSimple.Direction.REVERSE;
            }
        };
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        if (!DRIVE_CLASS.equals(MecanumDrive.class)) {
            throw new RuntimeException("Unsupported drive class: " + DRIVE_CLASS.getName());
        }

        DriveViewFactory dvf = hardwareMap -> {
            MecanumDrive md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            List<EncoderGroup> encoderGroups = new ArrayList<>();
            List<EncoderRef> leftEncs  = new ArrayList<>();
            List<EncoderRef> rightEncs = new ArrayList<>();
            List<EncoderRef> parEncs   = new ArrayList<>();
            List<EncoderRef> perpEncs  = new ArrayList<>();
            LazyImu lazyImu;

            if (md.localizer instanceof PinpointLocalizer) {
                PinpointView pv = makePinpointView((PinpointLocalizer) md.localizer);
                encoderGroups.add(new PinpointEncoderGroup(pv));
                parEncs.add(new EncoderRef(0, 0));
                perpEncs.add(new EncoderRef(0, 1));
                lazyImu = new PinpointIMU(pv);
            } else {
                throw new RuntimeException("Unknown localizer: " + md.localizer.getClass().getName());
            }

            return new DriveView(
                    DriveType.MECANUM,
                    MecanumDrive.PARAMS.inPerTick,
                    MecanumDrive.PARAMS.maxWheelVel,
                    MecanumDrive.PARAMS.minProfileAccel,
                    MecanumDrive.PARAMS.maxProfileAccel,
                    encoderGroups,
                    Arrays.asList(md.leftFront, md.leftBack),
                    Arrays.asList(md.rightFront, md.rightBack),
                    leftEncs,
                    rightEncs,
                    parEncs,
                    perpEncs,
                    lazyImu,
                    md.voltageSensor,
                    () -> new MotorFeedforward(
                            MecanumDrive.PARAMS.kS,
                            MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                            MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick),
                    0
            );
        };

        manager.register(metaForClass(AngularRampLogger.class),           new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class),             new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class),           new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class),             new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class),           new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class),      new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class),  new DeadWheelDirectionDebugger(dvf));

        manager.register(metaForClass(ManualFeedbackTuner.class),  ManualFeedbackTuner.class);
        manager.register(metaForClass(SplineTest.class),           SplineTest.class);
        manager.register(metaForClass(LocalizationTest.class),     LocalizationTest.class);

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
