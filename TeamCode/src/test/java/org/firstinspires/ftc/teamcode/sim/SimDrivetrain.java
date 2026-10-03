package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.math.Vector;

/**
 * A holonomic point robot standing in for Pedro's {@code Mecanum}. Instead of wheel powers,
 * {@link #calculateDrive} returns the field-frame command {@code {vx, vy, omega, 0}} as
 * fractions of full power; {@link SimLocalizer} integrates it. No inertia — the robot moves at
 * the commanded fraction of {@link #MAX_SPEED} immediately — which is enough to check that
 * routines terminate and commands hand the drivetrain over correctly, not to tune PIDF.
 */
public final class SimDrivetrain extends Drivetrain {

    /** in/s at full power. */
    public static final double MAX_SPEED = 60;

    /** rad/s at full power. */
    public static final double MAX_TURN_RATE = 2 * Math.PI;

    private double[] command = new double[4];
    private boolean teleop;

    public SimDrivetrain() {
        maxPowerScaling = 1.0;
    }

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower,
                                   Vector pathingPower, double robotHeading) {
        Vector translation = correctivePower.plus(pathingPower);
        if (translation.getMagnitude() > maxPowerScaling) translation.setMagnitude(maxPowerScaling);
        // Mecanum adds the heading vector to the right side and subtracts it from the left, so
        // its component along the robot's heading is the turn command.
        double turn = headingPower.getXComponent() * Math.cos(robotHeading)
                + headingPower.getYComponent() * Math.sin(robotHeading);
        turn = Math.max(-maxPowerScaling, Math.min(maxPowerScaling, turn));
        return new double[] {translation.getXComponent(), translation.getYComponent(), turn, 0};
    }

    @Override
    public void runDrive(double[] drivePowers) {
        command = drivePowers.clone();
    }

    @Override
    public void breakFollowing() {
        // Pedro calls this itself at every followPath/holdPoint/startTeleopDrive, so a count of
        // calls says nothing about who stopped the robot — assert on motion instead.
        command = new double[4];
    }

    @Override public void updateConstants() {}
    @Override public void startTeleopDrive() { teleop = true; }
    @Override public void startTeleopDrive(boolean brakeMode) { teleop = true; }
    @Override public double xVelocity() { return MAX_SPEED; }
    @Override public double yVelocity() { return MAX_SPEED; }
    @Override public void setXVelocity(double xMovement) {}
    @Override public void setYVelocity(double yMovement) {}
    @Override public double getVoltage() { return 12.0; }

    @Override
    public String debugString() {
        return String.format("SimDrivetrain{vx=%.2f vy=%.2f w=%.2f}", command[0], command[1], command[2]);
    }

    // --- for assertions ---

    double vx() { return command[0] * MAX_SPEED; }
    double vy() { return command[1] * MAX_SPEED; }
    double omega() { return command[2] * MAX_TURN_RATE; }

    public boolean teleopStarted() { return teleop; }

    public boolean isCommandingMotion() {
        return command[0] != 0 || command[1] != 0 || command[2] != 0;
    }
}
