package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by sopa on 12/7/16.
 */
@Autonomous(name = "TestCorrection", group = "Autonomous")
public class TestCorrection extends AutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initalize();
        waitForStart();
        telemetry.addData("before yaw forward", getGryoYaw());
        sleep(3000);
        moveForwardWithCorrection(.15, 3000);
        telemetry.addData("after yaw forward", getGryoYaw());
        sleep(3000);
        telemetry.addData("before yaw backward", getGryoYaw());
        moveBackWarddWithCorrection(.15, 3000);
        sleep(3000);
        telemetry.addData("after yaw backward", getGryoYaw());
        sleep(3000);
    }
}
