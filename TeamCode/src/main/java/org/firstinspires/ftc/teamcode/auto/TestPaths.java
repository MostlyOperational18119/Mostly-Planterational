package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * Paths for {@link TestAuto}: 24 in straight out along +x and straight back, heading held at
 * zero. Placeholder geometry — enough room to test on any practice floor. Built in the
 * constructor, before {@code waitForStart()}, never mid-routine (architecture doc §8.1).
 */
public final class TestPaths {

    public static final Pose START = new Pose(0, 0, 0);
    public static final Pose OUT = new Pose(24, 0, 0);

    public final PathChain out, back;

    public TestPaths(Follower f) {
        out = f.pathBuilder()
                .addPath(new BezierLine(START, OUT))
                .setConstantHeadingInterpolation(START.getHeading())
                .build();
        back = f.pathBuilder()
                .addPath(new BezierLine(OUT, START))
                .setConstantHeadingInterpolation(START.getHeading())
                .build();
    }
}
