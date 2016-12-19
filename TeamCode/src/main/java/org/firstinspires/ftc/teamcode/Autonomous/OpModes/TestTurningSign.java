package org.firstinspires.ftc.teamcode.Autonomous.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by sopa on 12/18/16.
 */
@Autonomous(name ="TestTurningSign", group ="Autonomous")
public class TestTurningSign extends AutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();
        telemetry.addData("test1", "init");
        telemetry.update();
        testTurningNegative(.2, 30);
    }
}
