package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by 4546 on 12/21/16.
 * This auto will move to the beacons, hit both, turn, shoot, and then knock the cap ball
 * Test Count: 12 +
 */
@Autonomous(name = "BlueIsTheWarmestColor", group = "Autonomous")
public class BlueSideScore extends AutoOpMode {
    public BlueSideScore() { super(); };
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("init", "test1");
        telemetry.update();
        long beforeTime = System.currentTimeMillis();
        double beforeAngle = getGyroYaw();
        while(System.currentTimeMillis() - beforeTime < 15000) {
            telemetry.addData("angle", Math.abs(getGyroYaw() - beforeAngle));
            telemetry.update();
        }
        waitForStart();
        moveBackwardPID(2500);
        //correct(beforeAngle);
        moveBackwardsToWhiteLine(300);
        beaconValue();
        moveForwardPID(2300);
        moveForwardsToWhiteLine(600);
        turnLeftWithPID(15);
        moveForwardWithEncoders(.25, 1000);
        //bring down shooter
        bringDownShooter(.1, 1100);
        sleep(750);
        //shoot
        shoot(.85, 350);
        moveForwardWithEncoders(.4, 3000);
//        moveForwardPID(500);
//        //bring down shooter
//        bringDownShooter(.1, 1100);
//        sleep(750);
//        //shoot
//        shoot(.85, 400);
//        sleep(750);
//        turnRightWithPID(50);
//        sleep(500);
//        moveForwardPID(2500);
//        sleep(500);
//        turnLeftWithPID(50);
//        sleep(500);
//        correct(beforeAngle);
//        //moveToWhiteLine();

    }
}
