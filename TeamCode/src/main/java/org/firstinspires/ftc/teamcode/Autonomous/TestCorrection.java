package org.firstinspires.ftc.teamcode.Autonomous;

/**
 * Created by sopa on 12/7/16.
 */
public class TestCorrection extends AutoOpMode{
    @Override
    public void runOpMode() throws InterruptedException
    {
        init();
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
