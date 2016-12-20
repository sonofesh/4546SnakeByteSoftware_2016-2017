package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/19/16.
 */
@Autonomous(name = "PID turning", group = "Autonomous")
public class PIDTurning extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("test1", "init");
        telemetry.update();
        waitForStart();
        turnRightWithPID(90);
        sleep(10000);
    }
}
