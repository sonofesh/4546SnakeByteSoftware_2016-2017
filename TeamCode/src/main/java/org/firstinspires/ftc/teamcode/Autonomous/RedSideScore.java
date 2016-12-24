package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/19/16.
 */
@Autonomous(name = "BloodDiamond", group = "Autonomous")
public class RedSideScore extends AutoOpMode {
    public RedSideScore() {
        super();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("init", "test17");
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
        moveForwardPID(3050);
        sleep(500);
        correct(perpendicular);
        sleep(500);
        beaconValue();
        sleep(500);
        //correct(perpendicular);
        moveToWhiteLine(300);
        sleep(1000);
        moveForwardPID(2000);
        sleep(1000);
        moveToWhiteLine(600);
        beaconValue();
        moveBackwardsWithATiltRight(.4, 2500);
    }
}
