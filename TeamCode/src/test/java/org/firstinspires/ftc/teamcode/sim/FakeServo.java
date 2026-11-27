package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A {@link Servo} that records every {@code setPosition} — every hardware transaction the
 * write gate let through. Every other method is a no-op returning a zero value. A
 * {@link Proxy}, like {@link FakeMotor}.
 */
public final class FakeServo {

    public final Servo servo;
    private final List<Double> writes = new ArrayList<>();

    public FakeServo() {
        servo = (Servo) Proxy.newProxyInstance(
                Servo.class.getClassLoader(),
                new Class<?>[] {Servo.class},
                (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setPosition":
                            writes.add((Double) args[0]);
                            return null;
                        case "getPosition":
                            return position();
                        case "toString":
                            return "FakeServo";
                        case "hashCode":
                            return System.identityHashCode(proxy);
                        case "equals":
                            return proxy == args[0];
                        default:
                            Class<?> type = method.getReturnType();
                            if (type == boolean.class) return false;
                            if (type == int.class) return 0;
                            if (type == double.class) return 0.0;
                            return null;
                    }
                });
    }

    /** Every position actually sent to the hardware, in order. */
    public List<Double> writes() {
        return Collections.unmodifiableList(writes);
    }

    /** The last position written — NaN if never written. */
    public double position() {
        return writes.isEmpty() ? Double.NaN : writes.get(writes.size() - 1);
    }
}
