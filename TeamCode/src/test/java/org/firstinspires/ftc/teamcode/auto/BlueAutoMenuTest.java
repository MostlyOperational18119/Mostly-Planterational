package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.routines.ShootingRoutines;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/** Every menu option on the simulated robot, and moving through the menu. */
public class BlueAutoMenuTest {

    private double approachHive, toHive, decide, approach, park;
    private double fieldTurn, fieldCollect, fieldDwell, fieldLeg, fieldSettle;

    @Before
    public void shortenTheLegs() {
        // The sim runs in real time. Termination is what is asserted, so the legs need not
        // arrive; the timeouts end them.
        approachHive = ShootingRoutines.APPROACH_HIVE_TIMEOUT_MS;
        toHive = ShootingRoutines.TO_HIVE_TIMEOUT_MS;
        decide = ShootingRoutines.DECIDE_TIMEOUT_MS;
        approach = ShootingRoutines.APPROACH_TIMEOUT_MS;
        park = ShootingRoutines.PARK_TIMEOUT_MS;
        ShootingRoutines.APPROACH_HIVE_TIMEOUT_MS = 100;
        ShootingRoutines.TO_HIVE_TIMEOUT_MS = 100;
        ShootingRoutines.DECIDE_TIMEOUT_MS = 100;
        ShootingRoutines.APPROACH_TIMEOUT_MS = 100;
        ShootingRoutines.PARK_TIMEOUT_MS = 100;
        fieldTurn = ShootingRoutines.FIELD_TURN_TIMEOUT_MS;
        fieldCollect = ShootingRoutines.FIELD_COLLECT_TIMEOUT_MS;
        fieldDwell = ShootingRoutines.FIELD_COLLECT_DWELL_MS;
        fieldLeg = ShootingRoutines.FIELD_LEG_TIMEOUT_MS;
        fieldSettle = ShootingRoutines.FIELD_SHOOT_SETTLE_MS;
        ShootingRoutines.FIELD_TURN_TIMEOUT_MS = 100;
        ShootingRoutines.FIELD_COLLECT_TIMEOUT_MS = 100;
        ShootingRoutines.FIELD_COLLECT_DWELL_MS = 100;
        ShootingRoutines.FIELD_LEG_TIMEOUT_MS = 100;
        ShootingRoutines.FIELD_SHOOT_SETTLE_MS = 100;
    }

    @After
    public void restoreTheLegs() {
        ShootingRoutines.APPROACH_HIVE_TIMEOUT_MS = approachHive;
        ShootingRoutines.TO_HIVE_TIMEOUT_MS = toHive;
        ShootingRoutines.DECIDE_TIMEOUT_MS = decide;
        ShootingRoutines.APPROACH_TIMEOUT_MS = approach;
        ShootingRoutines.PARK_TIMEOUT_MS = park;
        ShootingRoutines.FIELD_TURN_TIMEOUT_MS = fieldTurn;
        ShootingRoutines.FIELD_COLLECT_TIMEOUT_MS = fieldCollect;
        ShootingRoutines.FIELD_COLLECT_DWELL_MS = fieldDwell;
        ShootingRoutines.FIELD_LEG_TIMEOUT_MS = fieldLeg;
        ShootingRoutines.FIELD_SHOOT_SETTLE_MS = fieldSettle;
    }

    @Test
    public void everyOptionBuildsFromItsOwnSidesPathsAndFinishes() {
        for (AutoOption option : AutoMenus.BLUE) {
            SimRobot sim = new SimRobot(new Pose(0, 0, 0));
            boolean[] decided = {false};
            Command routine = option.routine.build(sim.robot, Alliance.BLUE, Cell.BLUE_FAR,
                    new AutoPaths(option.side), shoot -> decided[0] = true);
            routine.schedule();

            assertTrue(option + " never finished",
                    sim.cycleUntil(() -> !routine.isScheduled(), 3000));
            assertTrue(option + " never decided whether to shoot", decided[0]);
        }
    }

    @Test
    public void theMenuWrapsAtBothEnds() {
        int size = AutoMenus.BLUE.size();
        assertEquals(size - 1, AutoMenus.step(0, -1, size));
        assertEquals(0, AutoMenus.step(size - 1, 1, size));
    }
}
