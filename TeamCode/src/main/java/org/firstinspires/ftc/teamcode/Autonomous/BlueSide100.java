package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/31/16.
 * Test count:
 * Shoot first, hit beacons sequentially, hit cap ball, park
 */

public class BlueSide100 extends AutoOpMode {
    public BlueSide100() { super(); };

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        initialize();
        telemetry.addData("init", "test7");
        telemetry.update();
        waitForStart();
        double perpendicular = getGyroYaw();
        int movement = 0;
        moveForward(.175, 500);
        //moveForwardPID(500);
        //bring down shooter
        bringDownShooter(.1, 1100);
        sleep(750);
        //shoot
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        shoot(.8, 350);
        sleep(750);
        //turnRightWithPID(50);
        turnLeftWithPID(50);
        sleep(500);
        moveForwardPID(.0002, .0000001, 0.0, 3250);
        sleep(500);
        correct(perpendicular, .004, .000015, 0.0);
        sleep(500);
        moveForwardsToWhiteLine(300);
//        if(pushBlueBeacon(100) == 0)
//            movement = 0;
//        else
//            movement = 100;
        sleep(1000);
        correct(perpendicular, .04, .00015, 0.0);
        moveForwardPID(2500 - movement);
        sleep(1000);
        movement = 0;
        moveForwardsToWhiteLine(600);
        beaconValue();
//        if(pushBlueBeacon(100) == 0)
//            movement = 0;
//        else
//            movement = 100;
        sleep(1000);
        moveBackwardsWithATiltLeft(.4, 3500 + movement);
    }
}
