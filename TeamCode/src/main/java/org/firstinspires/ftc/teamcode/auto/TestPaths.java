package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Paths for {@link TestAuto}: 24 in straight out along +x and straight back, heading held at
 * zero. Placeholder geometry — enough room to test on any practice floor. Built before
 * {@code waitForStart()}, never mid-routine (architecture doc §8.1).
 */
public final class TestPaths {

    public static final Pose START = new Pose(0, 0, 0);
    public static final Pose OUT = new Pose(24, 0, 0);

    public final Path out = Paths.line(START, OUT).constant(START);
    public final Path back = Paths.line(OUT, START).constant(START);
}
