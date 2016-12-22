package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/21/16.
 * This auto will move forward, shoot, hit both beacons, then knock ball and park on center
 * Goal 100 points
 * Before you start testing, run testTurningNegative
 */
@Autonomous(name = "BlueIsTheWarmestColor", group = "Autonomous")
public class BlueSideScore extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("init", "test12");
        telemetry.update();
        waitForStart();
        double perpendicular = getGyroYaw();
        moveForwardPID(500);
        //bring down shooter
        bringDownShooter(.1, 1100);
        sleep(750);
        //shoot
        shoot(.85, 400);
        sleep(750);
        turnRightWithPID(50);
        sleep(500);
        moveForwardPID(2500);
        sleep(500);
        turnLeftWithPID(50);
        sleep(500);
        correct(perpendicular);
        //moveToWhiteLine();
    }
}
