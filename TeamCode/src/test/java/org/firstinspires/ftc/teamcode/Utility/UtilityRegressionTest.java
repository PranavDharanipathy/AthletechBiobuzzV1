package org.firstinspires.ftc.teamcode.Utility;

import org.junit.Test;

/** Includes the standalone Java 8 utility checks in Gradle's unit-test gate. */
public class UtilityRegressionTest {
    @Test public void integratorReset() {
        DynamicTrapezoidalSumTest.main(new String[0]);
    }

    @Test public void voltageFaultAndRecovery() {
        VoltageCompensatorTest.main(new String[0]);
    }
}
