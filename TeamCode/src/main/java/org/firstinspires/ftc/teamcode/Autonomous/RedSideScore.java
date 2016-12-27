package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by 4546 Snakebyte on 12/19/16.
 * Test count: 18 +
 * This will essentially be our red side auto, provided the first turn is reversed
 * DELETE WHEN DONE.
 */
@Autonomous(name = "BloodDiamond", group = "Autonomous")
public class RedSideScore extends AutoOpMode {
    public RedSideScore() {
        super();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("init", "test3");
        telemetry.update();
        waitForStart();
        double perpendicular = getGyroYaw();
        moveForwardPID(500);
        //bring down shooter
        bringDownShooter(.1, 1100);
        sleep(750);
        //shoot
        shoot(.85, 350);
        sleep(750);
        turnRightWithPID(50);
        sleep(500);
        moveForwardPID(3050);
        sleep(500);
        correct(perpendicular);
        sleep(500);
        beaconValue();
        sleep(500);
        moveBackwardsToWhiteLine(300);
        sleep(1000);
        moveForwardPID(2300);
        sleep(1000);
        moveForwardsToWhiteLine(600);
        beaconValue();
        sleep(2000);
        moveBackwardsWithATiltRight(.4, 3500);
    }
}
