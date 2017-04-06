package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by viperbots on 4/4/2017.
 */
@Autonomous (name = "TestRange", group = "Autonomous")
public class TestRange extends AutoOpMode {

    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();
        telemetry.addData("Before Angle", getGyroYaw());
        telemetry.update();
        sleep(1000);
        double angle = getGyroYaw();
        telemetry.addData("After Angle", angle);
        telemetry.addData("Raw Distance", getRawDistance());
        telemetry.addData("Trig Distance", getTrigDistance(angle));
        telemetry.addData("Distance to Wall", getDistanceToWall(angle));
        telemetry.update();
        sleep(3000);
        angle = getGyroYaw();
        telemetry.addData("After Angle", angle);
        telemetry.addData("Raw Distance", getRawDistance());
        telemetry.addData("Trig Distance", getTrigDistance(angle));
        telemetry.addData("Distance to Wall", getDistanceToWall(angle));
        telemetry.update();
    }
}
