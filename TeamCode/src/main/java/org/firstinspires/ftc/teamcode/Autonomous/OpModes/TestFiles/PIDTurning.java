package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/19/16.
 * Notes: For 90˚ proportion of .005 is perfect but doesn't work for smaller turns
 */
@Autonomous(name = "PID turning", group = "Autonomous")
@Disabled
public class PIDTurning extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("test22", "init");
        telemetry.update();
        waitForStart();
        telemetry.addData("beforeAngle", getGyroYaw());
        telemetry.update();
        turnRightWithPID(90);
        long lastTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - lastTime < 5000){
        }
        turnRightWithPID(30);
        lastTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - lastTime < 5000){
        }
    }
}
