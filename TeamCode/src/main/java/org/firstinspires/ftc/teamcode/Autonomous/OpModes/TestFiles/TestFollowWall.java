package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by viperbots on 1/11/2017.
 */

@Autonomous(name = "TestCorrection", group = "Autonomous")
@Disabled
public class TestFollowWall extends AutoOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.log();
        initialize();
        telemetry.addData("test1", "init");
        telemetry.update();
        waitForStart();
        telemetry.addData("before yaw forward", getGyroYaw());
        telemetry.update();
        sleep(1000);
        telemetry.addData("before Distance", getRawDistance());
        telemetry.update();
        sleep(2000);
        followWallBlue(2000,10, getGyroYaw());
        sleep(1000);
        telemetry.addData("after yaw", getGyroYaw());
        telemetry.update();
        sleep(1000);
        telemetry.addData("after distance", getRawDistance());
        telemetry.update();
        sleep(1000);
        moveBackwardPID(.0003, .0000003, 0.0, 2000);
    }
}
