package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * The real {@link HubManager}: every hub in {@code MANUAL} bulk-caching mode, one cache clear
 * per loop.
 *
 * <p>{@code MANUAL} rather than {@code AUTO} is deliberate. {@code AUTO} invalidates the cache
 * on the second read of the same register, silently reintroducing ~2 ms round-trips;
 * {@code MANUAL} guarantees exactly one bulk transfer per hub per loop (architecture doc §4.3).
 */
public final class LynxHubs implements HubManager {

    private final List<LynxModule> hubs;

    public LynxHubs(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (int i = 0; i < hubs.size(); i++) {
            hubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void startCycle() {
        // Indexed loop: no Iterator allocation on the hot path.
        for (int i = 0; i < hubs.size(); i++) {
            hubs.get(i).clearBulkCache();
        }
    }

    public int hubCount() {
        return hubs.size();
    }
}
