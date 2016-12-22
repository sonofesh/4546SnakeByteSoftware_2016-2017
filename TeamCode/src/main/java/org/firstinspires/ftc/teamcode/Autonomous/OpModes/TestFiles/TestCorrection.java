package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/7/16.
 * Test log: 28 + 25
 */
@Autonomous(name = "TestCorrection", group = "Autonomous")
public class TestCorrection extends AutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();
        telemetry.addData("test32", "init");
        telemetry.update();
        waitForStart();
        telemetry.addData("before yaw forward", getGyroYaw());
        telemetry.update();
        telemetry.update();
        sleep(3000);
        moveBackwardPID(2000);
        sleep(5000);
        moveForwardPID(2000);
        long lastTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - lastTime < 7000){
        }

//        telemetry.addData("before yaw backward", getGyroYaw());
//        telemetry.update();
//        moveBackWardWithCorrection(.15, 2000);
//        //hacked
//        sleep(5000);
//        telemetry.addData("after yaw backward", getGyroYaw());
//        telemetry.update();
//        sleep(5000);
//        telemetry.addData("difference in angle", (getGyroYaw() - beforeAngle));
//        telemetry.update();
    }
}
