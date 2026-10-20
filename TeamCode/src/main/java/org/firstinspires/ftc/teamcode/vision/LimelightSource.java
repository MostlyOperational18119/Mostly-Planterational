package org.firstinspires.ftc.teamcode.vision;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * The Limelight 3A behind {@link VisionSource}. The only class that touches
 * {@code Limelight3A} or {@code LLResult}.
 *
 * <p>Per-cycle calls are exactly the two CLAUDE.md allows on the control thread:
 * {@code getLatestResult()} (a volatile read — the driver polls on its own thread) and
 * {@code isConnected()} (clock arithmetic). Every other {@code Limelight3A} method is a
 * blocking HTTP call and appears only in {@link #fromHardwareMap}, at init.
 *
 * <p>No botpose (MegaTag1 or 2). Every BIOBUZZ tag is on a HIVE CELL that tips during the
 * match, so a field-map solve is only right while each HIVE sits as the map assumes. Instead
 * each tag's position relative to the robot ({@code getTargetPoseRobotSpace()}) is passed on,
 * and {@link ClusterFit} turns a CELL's tags into a cluster centre.
 *
 * <p>Robot-relative poses are robot poses only if the camera's mounting (forward, right, up,
 * roll, pitch, yaw) is entered in the Limelight UI; z is then height above the floor.
 */
@Configurable
public final class LimelightSource implements VisionSource {

    /** Name in the robot configuration. */
    public static final String NAME = "limelight";

    /** The AprilTag pipeline on the camera: 36h11, full 3D, tag size 3.25 in (82.55 mm). */
    private static final int PIPELINE = 0;

    /**
     * Maps the Limelight's robot-space y onto ours (Pedro's: + left). −1 assumes the Limelight's
     * y is + right. The SDK passes the camera's {@code t6t_rs} through unchanged (checked in the
     * SDK 12.0 classes), so this is purely the Limelight's convention. <b>Unverified</b>: with a
     * CELL to the robot's left, telemetry's {@code left} must read positive; if it reads
     * negative, aiming turns <em>away</em> from the target — flip this. Panels-editable so it can
     * be flipped live; it applies from the next frame.
     */
    public static double LEFT_SIGN = -1;

    /**
     * Which axes {@code t6t_rs} actually arrives in. Limelight's docs say robot space is
     * x forward / y right / z up, but on our camera it reads like <em>camera</em> space —
     * x right / y down / z forward — with the mounting still applied: with a CELL straight
     * ahead, the docs' mapping put its height in "left" (a +90° aim error) and its distance in
     * "up". True (the default) maps forward = z, right = x, up = −y. Panels-editable; if the
     * telemetry's fwd/left/up stop matching a tape measure, flip it and compare.
     */
    public static boolean CAMERA_STYLE_AXES = true;

    private final Limelight3A limelight;

    private LLResult lastResult;
    private VisionSample lastSample;

    /**
     * Init only — {@code pipelineSwitch} blocks up to 100 ms. Returns {@link VisionSource#NONE}
     * if there is no {@code "limelight"} in the configuration, so a robot without a camera
     * still runs (losing vision costs accuracy, not the match — §7.5).
     */
    public static VisionSource fromHardwareMap(HardwareMap hardwareMap) {
        Limelight3A limelight = hardwareMap.tryGet(Limelight3A.class, NAME);
        if (limelight == null) return VisionSource.NONE;
        limelight.setPollRateHz(100);       // must precede start(); ignored once running
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
        return new LimelightSource(limelight);
    }

    private LimelightSource(Limelight3A limelight) {
        this.limelight = limelight;
    }

    @Override
    public VisionSample latest() {
        LLResult result = limelight.getLatestResult();
        if (result == null) return null;
        // The driver replaces the result on every poll (≤100 Hz); convert each one once.
        if (result != lastResult) {
            lastResult = result;
            lastSample = convert(result);
        }
        return lastSample;
    }

    @Override
    public boolean online() {
        return limelight.isConnected();
    }

    @Override
    public void close() {
        limelight.stop();
    }

    private static VisionSample convert(LLResult r) {
        List<LLResultTypes.FiducialResult> fiducials = r.getFiducialResults();
        int n = fiducials.size();
        int[] ids = new int[n];
        double[] x = new double[n], y = new double[n], z = new double[n];
        for (int i = 0; i < n; i++) {
            LLResultTypes.FiducialResult f = fiducials.get(i);
            Position p = f.getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.INCH);
            ids[i] = f.getFiducialId();
            if (CAMERA_STYLE_AXES) {
                x[i] = p.z;
                y[i] = LEFT_SIGN * p.x;
                z[i] = -p.y;
            } else {
                x[i] = p.x;
                y[i] = LEFT_SIGN * p.y;
                z[i] = p.z;
            }
        }
        return new VisionSample(r.isValid(), r.getControlHubTimeStamp(),
                r.getCaptureLatency() + r.getTargetingLatency(), ids, x, y, z);
    }
}
