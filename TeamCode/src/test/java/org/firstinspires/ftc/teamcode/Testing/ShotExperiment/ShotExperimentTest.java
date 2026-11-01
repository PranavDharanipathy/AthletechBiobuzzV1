package org.firstinspires.ftc.teamcode.Testing.ShotExperiment;

import org.junit.Test;
import static org.junit.Assert.*;

public class ShotExperimentTest {
    private static ShotGate gate() {
        return new ShotGate(new ShotGate.Settings(75,400,.35,.1,10.5,.12,.8));
    }
    private static final class Rig {
        final ShotGate gate=gate(); double now;
        void tick(boolean trigger) { tick(trigger,3700,12,true,true,true,.02); }
        void tick(boolean trigger,double rpm,double volts,boolean armed,boolean geometry,boolean still,double dt) {
            now+=dt; gate.update(now,3700,rpm,volts,armed,geometry,still,trigger);
        }
        void ready() { for(int i=0;i<30;i++) tick(false); assertEquals(ShotGate.State.READY,gate.state()); }
        void feed() { ready(); tick(true); assertEquals(ShotGate.State.FEEDING,gate.state()); assertTrue(gate.feed()); }
    }
    @Test public void vacuumMatchesIndependentEndpoint() {
        double v=PollenPhysics.vacuumSpeed(6*.3048,54),theta=Math.toRadians(54);
        double time=6*.3048/(v*Math.cos(theta));
        assertEquals(59.55*.0254,PollenPhysics.releaseHeight(54)+v*Math.sin(theta)*time-9.80665*time*time/2,1e-12);
        assertEquals(6.083, v,.002);
        assertEquals(3227,PollenPhysics.wheelRpm(v,.5),1);
        assertEquals(9.883,PollenPhysics.vacuumSpeed(6*.3048,39.4),.003);
    }
    @Test public void geometryAndUnitsRejectUnsupportedInputs() {
        assertTrue(Double.isNaN(PollenPhysics.vacuumSpeed(0,54)));
        assertTrue(Double.isNaN(PollenPhysics.vacuumSpeed(2,60)));
        assertTrue(Double.isNaN(PollenPhysics.vacuumSpeed(2,Double.NaN)));
        assertTrue(Double.isNaN(PollenPhysics.wheelRpm(6,0)));
        assertEquals(1.0,PollenPhysics.wheelRpm(6,.45)/PollenPhysics.wheelRpm(6,.5)*.9,1e-12);
    }
    @Test public void earlyPressIsNotQueued() {
        Rig r=new Rig(); for(int i=0;i<30;i++) r.tick(true);
        assertEquals(ShotGate.State.READY,r.gate.state()); assertFalse(r.gate.feed());
        r.tick(false); r.tick(true); assertTrue(r.gate.feed());
    }
    @Test public void freshReadyPressFeedsDirectlyWithoutServoDelay() {
        Rig r=new Rig(); r.ready(); r.tick(true);
        assertTrue(r.gate.feed()); assertEquals(1,r.gate.attempts());
        for(int i=0;i<5;i++) r.tick(false); assertTrue(r.gate.feed());
        for(int i=0;i<3;i++) r.tick(false); assertFalse(r.gate.feed());
    }
    @Test public void unstableSpeedPreventsMotorFeed() {
        Rig r=new Rig();r.ready();r.tick(true,3400,12,true,true,true,.02);
        assertFalse(r.gate.feed());assertEquals(0,r.gate.attempts());
    }
    @Test public void normalShotDipDoesNotCutPulseButRecoveryRequiresSpeed() {
        Rig r=new Rig();r.feed();r.tick(false,3000,12,true,true,true,.02);assertTrue(r.gate.feed());
        for(int i=0;i<10;i++) r.tick(false,3000,12,true,true,true,.02);
        assertFalse(r.gate.feed());
        for(int i=0;i<60;i++) r.tick(false,3000,12,true,true,true,.02);
        assertNotEquals(ShotGate.State.READY,r.gate.state());
        for(int i=0;i<30;i++) r.tick(false);assertEquals(ShotGate.State.READY,r.gate.state());
    }
    @Test public void loopGapClosesFeedAndConsumesButton() {
        Rig r=new Rig();r.feed();r.tick(true,3700,12,true,true,true,.2);
        assertFalse(r.gate.feed());for(int i=0;i<100;i++)r.tick(true);
        assertFalse(r.gate.feed());assertEquals(1,r.gate.attempts());
    }
    @Test public void globalInterlocksAbortAnActiveFeed() {
        for(int fault=0;fault<6;fault++) {
            Rig r=new Rig();r.feed();
            r.tick(false,fault==0?Double.NaN:3700,fault==1?10:12,fault!=2,fault!=3,fault!=4,fault==5?0:.02);
            assertFalse("fault "+fault,r.gate.feed());
        }
    }
    @Test public void targetChangeNeedsNewDwell() {
        Rig r=new Rig();r.ready();r.gate.update(r.now+.02,3800,3800,12,true,true,true,true);
        assertFalse(r.gate.feed());assertEquals(ShotGate.State.BLOCKED,r.gate.state());
    }
    @Test public void highAccelerationPreventsReadinessEvenInsideBand() {
        Rig r=new Rig();for(int i=0;i<50;i++)r.tick(false,i%2==0?3670:3730,12,true,true,true,.02);
        assertNotEquals(ShotGate.State.READY,r.gate.state());
    }
    @Test public void feedCancellationRetainsShotSpacing() {
        Rig r=new Rig();r.feed();r.tick(false,3700,12,false,true,true,.02);
        for(int i=0;i<25;i++)r.tick(false);assertFalse(r.gate.feed());assertNotEquals(ShotGate.State.READY,r.gate.state());
    }
    @Test public void invalidConfigurationRejected() {
        try { new ShotGate.Settings(75,400,.35,.2,10.5,.12,.8);fail(); }
        catch(IllegalArgumentException expected) { }
        try { new ShotGate.Settings(Double.NaN,400,.35,.1,10.5,.12,.8);fail(); }
        catch(IllegalArgumentException expected) { }
    }
    @Test public void motorPowerIsVoltageBasedBoundedAndSlewLimited() {
        ExperimentFlywheel c=new ExperimentFlywheel();double previous=0,p=0;
        for(int i=0;i<100;i++) {
            p=c.update(3700,3700,12,.02);
            assertTrue(p>=0&&p<=.9);assertTrue(Math.abs(p-previous)<=.01600001);previous=p;
        }
        assertEquals(3700.0/6000,p,1e-12);
        double sag=c.update(3700,3700,11,.02);assertTrue(sag>p);
        assertEquals(0,c.update(0,3700,12,.02),0);
    }
    @Test public void motorRejectsInvalidMeasurementsAndCannotReverse() {
        ExperimentFlywheel c=new ExperimentFlywheel();
        assertEquals(0,c.update(3700,Double.NaN,12,.02),0);
        assertEquals(0,c.update(3700,0,9,.02),0);
        assertEquals(0,c.update(3700,0,12,.2),0);
        for(int i=0;i<1000;i++)assertEquals(0,c.update(500,9000,12,.02),0);
    }
    @Test public void motorPlantRecoversAfterShotAndBatterySag() {
        ExperimentFlywheel c=new ExperimentFlywheel();double rpm=0;
        for(int i=0;i<4000;i++) {
            double battery=i<1500?12:11;
            if(i==1600)rpm*=.8;
            double p=c.update(3700,rpm,battery,.02);
            // Independent first-order illustrative loaded motor; not a robot characterization.
            double steady=5600*p*battery/12;
            rpm+=(steady-rpm)*(-Math.expm1(-.02/.2));
        }
        assertEquals(3700,rpm,15);
    }
    @Test public void excessiveVoltageBlocksRelease() {
        Rig r=new Rig();r.ready();r.tick(true,3700,15,true,true,true,.02);
        assertFalse(r.gate.feed());assertEquals(0,r.gate.attempts());
    }
    @Test public void unreachableMotorTargetNeverBecomesReady() {
        ExperimentFlywheel motor=new ExperimentFlywheel();ShotGate gate=gate();double rpm=0,now=0;
        for(int i=0;i<1000;i++) {
            now+=.02;
            double p=motor.update(5000,rpm,10.5,.02);
            rpm+=(4800*p*10.5/12-rpm)*(-Math.expm1(-.02/.4));
            gate.update(now,5000,rpm,10.5,true,true,true,i%2==0);
            assertFalse(gate.feed());assertTrue(p<=.9);
        }
        assertEquals(0,gate.attempts());
    }

}
