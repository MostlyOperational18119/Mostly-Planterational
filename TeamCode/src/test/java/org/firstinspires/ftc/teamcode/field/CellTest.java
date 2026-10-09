package org.firstinspires.ftc.teamcode.field;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;

import org.junit.Test;

/** Tag assignments from the game manual, §9.9 / Fig. 9-17. */
public class CellTest {

    @Test
    public void tagIdsMatchTheManual() {
        int[][] expected = {{30, 33}, {34, 37}, {38, 41}, {42, 45}};
        Cell[] cells = {Cell.RED_FAR, Cell.RED_AUDIENCE, Cell.BLUE_AUDIENCE, Cell.BLUE_FAR};
        for (int c = 0; c < cells.length; c++) {
            for (int id = expected[c][0]; id <= expected[c][1]; id++) {
                assertSame("tag " + id, cells[c], Cell.forTag(id));
            }
        }
        assertNull(Cell.forTag(29));
        assertNull(Cell.forTag(46));
    }

    @Test
    public void partnersAreTheOtherEndOfTheSameHive() {
        assertSame(Cell.RED_AUDIENCE, Cell.RED_FAR.partner());
        assertSame(Cell.RED_FAR, Cell.RED_AUDIENCE.partner());
        assertSame(Cell.BLUE_FAR, Cell.BLUE_AUDIENCE.partner());
        for (Cell c : Cell.values()) assertEquals(c.alliance, c.partner().alliance);
    }
}
