package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

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
        testTurningNegative(.25, 30);
        sleep(5000);
    }
}
