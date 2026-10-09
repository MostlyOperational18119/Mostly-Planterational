package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.vision.VisionSample;
import org.firstinspires.ftc.teamcode.vision.VisionSource;

import java.util.ArrayList;
import java.util.List;

/** A camera the test controls: set {@link #online} and {@link #sample} directly. */
public final class FakeVisionSource implements VisionSource {

    public boolean online;
    public VisionSample sample;
    public boolean closed;

    @Override public VisionSample latest() { return sample; }
    @Override public boolean online() { return online; }
    @Override public void close() { closed = true; }

    /** Builds a frame tag by tag, as the camera would report it (robot frame, inches). */
    public static final class Frame {
        private final List<double[]> tags = new ArrayList<>();
        private long parsedAtMs;
        private double latencyMs;

        public Frame(long parsedAtMs, double latencyMs) {
            this.parsedAtMs = parsedAtMs;
            this.latencyMs = latencyMs;
        }

        public Frame tag(int id, double x, double y, double z) {
            tags.add(new double[] {id, x, y, z});
            return this;
        }

        /**
         * {@code cell}'s cluster centred at {@code (x, y, z)} with the row running along
         * {@code (ux, uy, uz)} (unit), including only {@code indices} (0–3) of its tags.
         */
        public Frame cluster(Cell cell, double x, double y, double z,
                             double ux, double uy, double uz, int... indices) {
            for (int i : indices) {
                double o = Cell.tagOffset(i);
                tag(cell.firstTagId + i, x + o * ux, y + o * uy, z + o * uz);
            }
            return this;
        }

        /** The whole cluster, row running left-right across the robot's view. */
        public Frame cluster(Cell cell, double x, double y, double z) {
            return cluster(cell, x, y, z, 0, 1, 0, 0, 1, 2, 3);
        }

        public VisionSample build() {
            int n = tags.size();
            int[] ids = new int[n];
            double[] x = new double[n], y = new double[n], z = new double[n];
            for (int i = 0; i < n; i++) {
                double[] t = tags.get(i);
                ids[i] = (int) t[0];
                x[i] = t[1];
                y[i] = t[2];
                z[i] = t[3];
            }
            return new VisionSample(true, parsedAtMs, latencyMs, ids, x, y, z);
        }
    }
}
