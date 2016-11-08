package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by sopa on 9/24/16.
 */

@Autonomous(name = "TestEncoders", group = "Autonomous")
public class TestEncoders extends AutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        moveForwardWithEncoders(.5, 500);
        moveBackWardWithEncoders(.5, 500);
        idle();
    }
}
