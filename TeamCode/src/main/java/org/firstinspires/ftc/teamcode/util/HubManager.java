package org.firstinspires.ftc.teamcode.util;

/**
 * Phase 1 of the loop: invalidate the Lynx bulk-read caches so that every sensor read in
 * phase 2 is served from one bulk transfer per hub (architecture doc §4.3).
 *
 * <p>This exists as an interface rather than a {@code List<LynxModule>} inside
 * {@link org.firstinspires.ftc.teamcode.Robot} because {@code LynxModule} is a concrete
 * Android class that cannot load on a plain JVM. Behind this seam, a {@code Robot} can be
 * constructed in a headless test with {@link #NONE}. See the Testing section of CLAUDE.md.
 */
public interface HubManager {

    /** Called once per loop, before any {@link Subsystem#read()}. */
    void startCycle();

    /** No hubs — for headless use. */
    HubManager NONE = () -> {};
}
