package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by 4546 Snakebyte on 12/19/16.
 * Test count: 18 + 3 +
 * This will essentially be our red side auto, provided the first turn is reversed
 * DELETE WHEN DONE.
 * Shoot first auto, configured for red side
 */
@Autonomous(name = "BloodDiamond", group = "Autonomous")
public class RedSideScore extends AutoOpMode {
    public RedSideScore() {
        super();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("init", "test1");
        telemetry.update();
        waitForStart();
        double perpendicular = getGyroYaw();
        int movement = 0;
        moveForwardPID(500);
        //bring down shooter
        bringDownShooter(.1, 1100);
        sleep(750);
        //shoot
        shoot(.85, 350);
        sleep(750);
        turnRightWithPID(50);
        //turnLeftWithPID(50);
        sleep(500);
        moveForwardPID(3050);
        sleep(500);
        correct(perpendicular);
        sleep(500);
        moveBackwardsToWhiteLine(300);
//        if(pushBlueBeacon(100) == 0)
//            movement = 0;
//        else
//            movement = 100;
        sleep(1000);
        moveForwardPID(2300 - movement);
        sleep(1000);
        movement = 0;
        moveForwardsToWhiteLine(600);
        beaconValue();
//        if(pushBlueBeacon(100) == 0)
//            movement = 0;
//        else
//            movement = 100;
        sleep(1000);
        moveBackwardsWithATiltRight(.4, 3500 + movement);
    }
}
