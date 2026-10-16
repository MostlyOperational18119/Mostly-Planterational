package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Test;

public class AimControllerTest {

    private static final double EPS = 1e-9;
    private final double launchDirection = AimController.LAUNCH_DIRECTION;
    private final double leftOffset = AimController.LAUNCHER_LEFT_OFFSET_IN;

    @After
    public void restoreTunables() {
        AimController.LAUNCH_DIRECTION = launchDirection;
        AimController.LAUNCHER_LEFT_OFFSET_IN = leftOffset;
    }

    @Test
    public void targetAheadNeedsNoTurnAndTargetLeftIsPositive() {
        assertEquals(0, AimController.error(60, 0), EPS);
        assertEquals(Math.PI / 4, AimController.error(40, 40), EPS);
        assertEquals(-Math.PI / 4, AimController.error(40, -40), EPS);
    }

    @Test
    public void aRearFacingLauncherTurnsTheBackToTheTarget() {
        AimController.LAUNCH_DIRECTION = Math.PI;
        assertEquals(0, Math.abs(AimController.error(-60, 0)), EPS);
        assertEquals(Math.PI, Math.abs(AimController.error(60, 0)), EPS);
    }

    @Test
    public void anOffsetLauncherIsAimedAlongItsOwnLine() {
        // Launcher 6 in left of centre: after turning by the error, a target at (60, 0) must
        // lie on the launcher's line y' = 6 in the rotated frame.
        AimController.LAUNCHER_LEFT_OFFSET_IN = 6;
        double e = AimController.error(60, 0);
        double yAfter = -60 * Math.sin(e);         // target's y in the turned robot frame
        assertEquals(6, yAfter, 1e-9);
        assertTrue("must turn right, putting the target on the launcher's left side", e < 0);
    }

    @Test
    public void turnHasADeadbandAKickAndACap() {
        assertEquals(0, AimController.turn(AimController.TOLERANCE / 2), EPS);
        double small = AimController.TOLERANCE * 2;
        assertEquals(AimController.KP * small + AimController.KS, AimController.turn(small), EPS);
        assertEquals(-(AimController.KP * small + AimController.KS), AimController.turn(-small), EPS);
        assertEquals(AimController.MAX_TURN, AimController.turn(Math.PI), EPS);
        assertEquals(-AimController.MAX_TURN, AimController.turn(-Math.PI), EPS);
        assertTrue(Double.isNaN(AimController.turn(Double.NaN)));
    }
}
