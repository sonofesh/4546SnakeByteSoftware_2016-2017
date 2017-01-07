package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/31/16.
 */
@Autonomous(name = "OneSideTurning", group = "Autonomous")
@Disabled
public class OneSideTurning extends AutoOpMode {
    public OneSideTurning() {super(); }
    //turnRightWithPID(50, .006, .00003, 0.0);
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("test1", "init");
        telemetry.update();
        waitForStart();
        turnLeftWithPIDOneSide(50, .006, .00003, 0.0);
        sleep(3000);
    }
}
