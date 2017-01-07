package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/26/16.
 * test count; 8 +
 */
@Autonomous(name = "Finish", group = "Autonomous")
@Disabled
public class Finish extends AutoOpMode {
    public Finish() {
        super();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("init", "test3");
        telemetry.update();
        waitForStart();
        moveBackwardsWithATiltLeft(.4, 3800);
    }
}
