package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import choreo.trajectory.SwerveSample;
import org.junit.jupiter.api.Test;

/**
 * Checks the left/right mirroring math in FollowPath.mirrorSample. This exact
 * transform shipped two real bugs (alpha not negated; module forces not
 * swapped), so it stays pinned by tests.
 */
class FollowPathTest {

    private static final double EPS = 1e-9;
    private static final double FIELD_WIDTH = 8.0;

    private static SwerveSample sample() {
        return new SwerveSample(
            1.25,               // t
            4.0,                // x
            1.5,                // y
            0.5,                // heading (rad)
            2.0,                // vx
            0.75,               // vy
            0.3,                // omega
            0.1,                // ax
            0.2,                // ay
            0.05,               // alpha
            new double[] { 1.0, 2.0, 3.0, 4.0 },   // module forces X (FL, FR, BL, BR)
            new double[] { 5.0, 6.0, 7.0, 8.0 }    // module forces Y
        );
    }

    @Test
    void mirrorReflectsYAndNegatesAllAngularQuantities() {
        SwerveSample mirrored = FollowPath.mirrorSample(sample(), FIELD_WIDTH);

        assertEquals(1.25, mirrored.t, EPS);
        assertEquals(4.0, mirrored.x, EPS);                  // X unchanged
        assertEquals(FIELD_WIDTH - 1.5, mirrored.y, EPS);    // Y reflected
        assertEquals(-0.5, mirrored.heading, EPS);           // heading negated
        assertEquals(2.0, mirrored.vx, EPS);                 // vx unchanged
        assertEquals(-0.75, mirrored.vy, EPS);               // vy negated
        assertEquals(-0.3, mirrored.omega, EPS);             // omega negated
        assertEquals(0.1, mirrored.ax, EPS);                 // ax unchanged
        assertEquals(-0.2, mirrored.ay, EPS);                // ay negated
        assertEquals(-0.05, mirrored.alpha, EPS);            // alpha negated too
    }

    @Test
    void mirrorSwapsLeftRightModulePairsAndNegatesYForces() {
        SwerveSample mirrored = FollowPath.mirrorSample(sample(), FIELD_WIDTH);

        // Reflection turns FL<->FR and BL<->BR (Choreo order FL, FR, BL, BR).
        assertArrayEquals(
            new double[] { 2.0, 1.0, 4.0, 3.0 },
            mirrored.moduleForcesX(),
            EPS
        );
        assertArrayEquals(
            new double[] { -6.0, -5.0, -8.0, -7.0 },
            mirrored.moduleForcesY(),
            EPS
        );
    }

    @Test
    void mirrorTwiceIsIdentity() {
        SwerveSample original = sample();
        SwerveSample roundTrip = FollowPath.mirrorSample(
            FollowPath.mirrorSample(original, FIELD_WIDTH),
            FIELD_WIDTH
        );

        assertEquals(original.y, roundTrip.y, EPS);
        assertEquals(original.heading, roundTrip.heading, EPS);
        assertEquals(original.vy, roundTrip.vy, EPS);
        assertEquals(original.omega, roundTrip.omega, EPS);
        assertEquals(original.alpha, roundTrip.alpha, EPS);
        assertArrayEquals(original.moduleForcesX(), roundTrip.moduleForcesX(), EPS);
        assertArrayEquals(original.moduleForcesY(), roundTrip.moduleForcesY(), EPS);
    }
}
