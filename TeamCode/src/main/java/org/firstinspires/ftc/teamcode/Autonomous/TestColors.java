package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by sopa on 10/10/16.
 */
@Autonomous(name = "TestColors", group = "Autonomous")
public class TestColors extends AutoOpMode

{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();
        waitForStart();
        getColors(colorSensorWL);
        composeTelemetry();
    }

}
