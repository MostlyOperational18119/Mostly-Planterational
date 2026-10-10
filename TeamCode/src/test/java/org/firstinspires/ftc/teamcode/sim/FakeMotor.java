package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A {@link DcMotorEx} that records every {@code setPower} — i.e. every hardware transaction
 * the write gate let through. Every other method is a no-op returning a zero value. A
 * {@link Proxy} rather than a hand-written class because {@code DcMotorEx} has ~50 methods.
 */
public final class FakeMotor {

    public final DcMotorEx motor;
    private final List<Double> writes = new ArrayList<>();

    public FakeMotor() {
        motor = (DcMotorEx) Proxy.newProxyInstance(
                DcMotorEx.class.getClassLoader(),
                new Class<?>[] {DcMotorEx.class},
                (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setPower":
                            writes.add((Double) args[0]);
                            return null;
                        case "getPower":
                            return writes.isEmpty() ? 0.0 : writes.get(writes.size() - 1);
                        case "toString":
                            return "FakeMotor";
                        case "hashCode":
                            return System.identityHashCode(proxy);
                        case "equals":
                            return proxy == args[0];
                        default:
                            return zero(method.getReturnType());
                    }
                });
    }

    /** Every power actually sent to the hardware, in order. */
    public List<Double> writes() {
        return Collections.unmodifiableList(writes);
    }

    /** The power the motor is running at — 0 if never written. */
    public double power() {
        return writes.isEmpty() ? 0.0 : writes.get(writes.size() - 1);
    }

    private static Object zero(Class<?> type) {
        if (type == boolean.class) return false;
        if (type == int.class) return 0;
        if (type == long.class) return 0L;
        if (type == double.class) return 0.0;
        if (type == float.class) return 0f;
        if (type == short.class) return (short) 0;
        if (type == byte.class) return (byte) 0;
        if (type == char.class) return '\0';
        return null;
    }
}
