package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/26/16.
 */
@Autonomous(name = "Finish", group = "Autonomous")
public class Finish extends AutoOpMode {
    public Finish() {
        super();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("init", "test8");
        telemetry.update();
        waitForStart();
        moveBackwardsWithATiltRight(.4, 3500);
    }
}
