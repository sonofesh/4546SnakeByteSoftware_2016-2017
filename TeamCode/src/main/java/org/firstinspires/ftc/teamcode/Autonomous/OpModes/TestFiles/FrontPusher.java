package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 1/16/17.
 */

@Autonomous(name = "FrontPusher", group = "Autonomous")
public class FrontPusher extends AutoOpMode {
    public FrontPusher() { super();}

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("test1", "init");
        telemetry.update();
        waitForStart();
        double perpendicular = getGyroYaw();
        pushFrontBlue(perpendicular);
        //pushFrontBlue(perpendicular);
    }
}
