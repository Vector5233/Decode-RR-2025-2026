package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/**
 * MecanumDrive — the central class that ties together the drivetrain motors,
 * the GoBilda Pinpoint localizer, and the RoadRunner motion controller.
 *
 * HOW IT WORKS (big picture):
 *   1. The localizer (PinpointLocalizer) reads the dead-wheel pod encoders and
 *      continuously estimates the robot's position (X, Y, heading) on the field.
 *   2. RoadRunner generates a time-parameterized trajectory — a sequence of
 *      (position, velocity) targets spaced along a smooth path.
 *   3. Each loop, HolonomicController compares where the robot IS (from the
 *      localizer) to where it SHOULD BE (from the trajectory) and computes a
 *      correction command.
 *   4. The feedforward model converts that command into motor powers that
 *      account for friction (kS), velocity (kV), and acceleration (kA).
 */
@Config
public final class MecanumDrive {

    // -------------------------------------------------------------------------
    // PARAMS — all the numbers you tune for your specific robot.
    // These are exposed to FTC Dashboard so you can change them live.
    // -------------------------------------------------------------------------
    public static class Params {

        // --- Encoder Scale ---
        // inPerTick: how many inches the robot travels for each encoder tick
        //   on the dead-wheel pods.  Measured empirically with the 24-inch test.
        // lateralInPerTick: same idea but for side-to-side (strafe) motion.
        //   Usually slightly different due to wheel scrub.
        // trackWidthTicks: the distance between the left and right drive wheels,
        //   expressed in TICKS (= physical track width in inches / inPerTick).
        //   Used to convert a turning command into individual wheel speeds.
        public double inPerTick          = 0.0011347;
        public double lateralInPerTick   = 0.0025312364686351473;
        public double trackWidthTicks    = 5090.462028683618;

        // --- Motor Feedforward ---
        // kS: the minimum power needed to overcome static friction (get the
        //   robot moving from a dead stop).  Units: motor power (0-1 scale).
        // kV: velocity gain — how much extra power per unit of desired speed.
        //   Higher kV → robot reaches target speed faster but may oscillate.
        // kA: acceleration gain — damps the response to sudden speed changes.
        public double kS = 1.6470288478121056;
        public double kV = 0.0005314850564224142;
        public double kA = 0.000111;

        // --- Motion Profile Limits ---
        // These cap how fast and how hard the robot accelerates along a path.
        // maxWheelVel: top speed in in/s for each wheel.
        // minProfileAccel / maxProfileAccel: deceleration and acceleration
        //   limits in in/s².  Keep minProfileAccel negative (braking).
        public double maxWheelVel      = 50;
        public double minProfileAccel  = -30;
        public double maxProfileAccel  = 30;

        // --- Turn Limits ---
        public double maxAngVel   = Math.PI;   // rad/s
        public double maxAngAccel = Math.PI;   // rad/s²

        // --- Path Controller Gains (PD controller) ---
        // axialGain:    how aggressively to correct forward/back position error.
        // lateralGain:  how aggressively to correct left/right position error.
        // headingGain:  how aggressively to correct heading (rotation) error.
        // *VelGain:     derivative term — damps oscillation by reacting to
        //               how quickly the error is changing.
        public double axialGain      = 5.0;
        public double lateralGain    = 5.0;
        public double headingGain    = 20.0;

        public double axialVelGain   = 0.1;
        public double lateralVelGain = 0.9;
        public double headingVelGain = 0.5;
    }

    public static Params PARAMS = new Params();

    // -------------------------------------------------------------------------
    // Kinematics — converts between robot-frame velocity commands and
    // individual wheel speeds.  Works in INCH units (inPerTick * trackWidthTicks
    // gives inches; inPerTick / lateralInPerTick gives the lateral correction).
    // -------------------------------------------------------------------------
    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks,
            PARAMS.inPerTick / PARAMS.lateralInPerTick);

    // Motion constraints derived from PARAMS
    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);

    public final VelConstraint defaultVelConstraint = new MinVelConstraint(Arrays.asList(
            kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
            new AngularVelConstraint(PARAMS.maxAngVel)));

    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public final VoltageSensor voltageSensor;
    public final Localizer localizer;

    // Telemetry log writers (sampled at 50 ms to avoid flooding the log)
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();
    private final DownsampledWriter estimatedPoseWriter  = new DownsampledWriter("ESTIMATED_POSE",  50_000_000);
    private final DownsampledWriter targetPoseWriter     = new DownsampledWriter("TARGET_POSE",     50_000_000);
    private final DownsampledWriter driveCommandWriter   = new DownsampledWriter("DRIVE_COMMAND",   50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------
    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {

        // Require up-to-date Lynx firmware to avoid known hardware bugs
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        // AUTO bulk-caching reads all encoder/sensor values in one I²C transaction
        // instead of one-per-read, dramatically speeding up the control loop.
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // --- Drive motors ---
        // Names must match the robot configuration in the Driver Station app.
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        // BRAKE holds position when power is 0 (prevents coasting past targets)
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse left-side motors so positive power = forward on all four wheels
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // --- Localizer ---
        // PinpointLocalizer wraps the GoBilda Pinpoint CPU, which reads the
        // two dead-wheel pods and outputs X/Y position + heading.
        localizer = new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, pose);

        // Zero the heading at robot power-on so autonomous always starts at 0°
        ((PinpointLocalizer) localizer).driver.setHeading(0, AngleUnit.RADIANS);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /**
     * Queries the localizer for the latest pose and velocity, and records it
     * in the pose history for dashboard display.  Call once per loop iteration.
     */
    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }
        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));
        return vel;
    }

    /**
     * Sends raw directional powers to the drive wheels — used for TeleOp.
     * Automatically normalizes so no wheel exceeds ±1.0 power.
     */
    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (com.acmerobotics.roadrunner.DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    /**
     * Forwards Pinpoint diagnostics to the OpMode telemetry display.
     */
    public void addLocalizerTelemetry(Telemetry telemetry) {
        ((PinpointLocalizer) localizer).addTelemetry(telemetry);
    }

    /**
     * Resets the Pinpoint's heading to zero.  Useful between autonomous phases.
     */
    public void resetHeading() {
        ((PinpointLocalizer) localizer).driver.setHeading(0, AngleUnit.RADIANS);
    }

    /**
     * Builds a trajectory starting from the given pose using the current
     * motion constraints.  Chain .lineToX(), .splineTo(), .turn() etc. on
     * the returned builder, then call .build() to get a runnable Action.
     */
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(1e-6, new ProfileParams(0.25, 0.1, 1e-2)),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint,
                defaultAccelConstraint);
    }

    // -------------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------------

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i++] = t.position.y;
        }
        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    /** Computes normalized motor power from a feedforward model and battery voltage. */
    private double[] computeWheelPowers(MecanumKinematics.WheelVelocities<Time> wheelVels, double voltage) {
        final MotorFeedforward ff = new MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick,
                PARAMS.kA / PARAMS.inPerTick);
        return new double[]{
                ff.compute(wheelVels.leftFront)  / voltage,
                ff.compute(wheelVels.leftBack)   / voltage,
                ff.compute(wheelVels.rightBack)  / voltage,
                ff.compute(wheelVels.rightFront) / voltage
        };
    }

    private void applyPowers(double[] p) {
        leftFront.setPower(p[0]);
        leftBack.setPower(p[1]);
        rightBack.setPower(p[2]);
        rightFront.setPower(p[3]);
    }

    private void addPinpointTelemetry(TelemetryPacket p) {
        PinpointLocalizer pl = (PinpointLocalizer) localizer;
        GoBildaPinpointDriver drv = pl.driver;
        if (drv.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
            p.put("PinX (in)",          drv.getPosX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
            p.put("PinY (in)",          drv.getPosY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
            p.put("PinHeading (deg)",   Math.toDegrees(drv.getHeading(org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit.RADIANS)));
            p.put("PinVelX (in/s)",     drv.getVelX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
            p.put("PinVelY (in/s)",     drv.getVelY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
            p.put("PinHeadingVel (deg/s)", Math.toDegrees(drv.getHeadingVelocity(org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit.RADIANS)));
        }
    }

    // -------------------------------------------------------------------------
    // FollowTrajectoryAction — executes a pre-built path
    // -------------------------------------------------------------------------

    /**
     * An Action (RoadRunner's async task unit) that drives the robot along a
     * TimeTrajectory.  Each call to run() advances one control loop iteration.
     * Returns true while the path is ongoing, false when it finishes.
     */
    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;
        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;
            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t = (beginTs < 0) ? (beginTs = Actions.now()) - beginTs : Actions.now() - beginTs;

            if (t >= timeTrajectory.duration) {
                applyPowers(new double[]{0, 0, 0, 0});
                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain)
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double[] powers = computeWheelPowers(wheelVels, voltageSensor.getVoltage());
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltageSensor.getVoltage(), powers[0], powers[1], powers[2], powers[3]));
            applyPowers(powers);

            Pose2d est = localizer.getPose();
            p.put("x",            est.position.x);
            p.put("y",            est.position.y);
            p.put("heading (deg)", Math.toDegrees(est.heading.toDouble()));
            addPinpointTelemetry(p);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);
            c.setStroke("#4CAF50"); Drawing.drawRobot(c, txWorldTarget.value());
            c.setStroke("#3F51B5"); Drawing.drawRobot(c, localizer.getPose());

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    // -------------------------------------------------------------------------
    // TurnAction — executes an in-place rotation
    // -------------------------------------------------------------------------

    /**
     * An Action that rotates the robot by a fixed angle.
     * Works the same way as FollowTrajectoryAction but uses a TimeTurn profile.
     */
    public final class TurnAction implements Action {
        private final TimeTurn turn;
        private double beginTs = -1;

        public TurnAction(TimeTurn turn) { this.turn = turn; }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t = (beginTs < 0) ? (beginTs = Actions.now()) - beginTs : Actions.now() - beginTs;

            if (t >= turn.duration) {
                applyPowers(new double[]{0, 0, 0, 0});
                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain)
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double[] powers = computeWheelPowers(wheelVels, voltageSensor.getVoltage());
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltageSensor.getVoltage(), powers[0], powers[1], powers[2], powers[3]));
            applyPowers(powers);

            Pose2d est = localizer.getPose();
            p.put("x",            est.position.x);
            p.put("y",            est.position.y);
            p.put("heading (deg)", Math.toDegrees(est.heading.toDouble()));
            addPinpointTelemetry(p);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);
            c.setStroke("#4CAF50"); Drawing.drawRobot(c, txWorldTarget.value());
            c.setStroke("#3F51B5"); Drawing.drawRobot(c, localizer.getPose());
            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }
}
