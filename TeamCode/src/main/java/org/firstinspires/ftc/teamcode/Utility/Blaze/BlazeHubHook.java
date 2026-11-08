package org.firstinspires.ftc.teamcode.Utility.Blaze;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Constants.BlazeConstants;

import dev.nextftc.hardware.lynx.NextLynxModule;
import dev.nextftc.robot.opmode.BlazeBulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.OpModeHook;

public final class BlazeHubHook implements OpModeHook {

    private final boolean control;

    public BlazeHubHook(boolean control) {
        this.control = control;
    }

    @Override
    public void afterConstruction(NextOpMode opMode) {

        if (!BlazeConstants.ENABLED || !BlazeSession.isOpen()) return;
        if (control ? !BlazeConstants.BULK_CONTROL_HUB : !BlazeConstants.BULK_EXPANSION_HUB) return;
        LynxModule hub = BlazeSession.find(control);
        if (hub == null) return;
        try {
            new BlazeBulkReadHook(control ? NextLynxModule.Type.CONTROL_HUB : NextLynxModule.Type.EXPANSION_HUB, control && BlazeConstants.CONTROL_HUB_PACKETS >= 2).afterConstruction(opMode);
            BlazeSession.markFed(control ? BlazeSession.CONTROL_HUB : BlazeSession.EXPANSION_HUB, hub);
        } catch (Throwable t) {
            RobotLog.ee("Blaze", t, (control ? "Control" : "Expansion") + " Hub bulk reads failed, carrying on without them");
        }
    }
}
