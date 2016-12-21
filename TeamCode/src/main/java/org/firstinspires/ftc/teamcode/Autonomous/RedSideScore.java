package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/19/16.
 */
public class RedSideScore extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("test1", "init");
        telemetry.update();
        double beforeTurnA = getGyroYaw();
        moveForwardWithCorrection(.25, 500);
        turnRightWithGyro(beforeTurnA + 22, 2.5);
        moveToWhiteLine(.2, 300);

    }
}
