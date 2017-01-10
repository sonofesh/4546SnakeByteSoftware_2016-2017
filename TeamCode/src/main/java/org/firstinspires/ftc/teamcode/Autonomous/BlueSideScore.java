package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by 4546 on 12/21/16.
 * This auto will move to the beacons, hit both, turn, shoot, and then knock the cap ball
 * Test Count: 12 +
 */
@Autonomous(name = "BlueIsTheWarmestColor", group = "Autonomous")
@Disabled
public class BlueSideScore extends AutoOpMode {
    public BlueSideScore() { super(); };
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        double power = .8;
        telemetry.addData("init", "test1");
        telemetry.update();
        waitForStart();
        double perpendicular = getGyroYaw();
        moveForward(.16, 500);
        //moveForwardPID(500);
        //bring down shooter
        bringDownShooter(.1, 1100);
        sleep(750);
        //shoot
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        if(voltage <= 13.5)
            power = .9;
        else if(voltage <= 13.75 && voltage > 13.5)
            power = .85;
        else if(voltage > 13.75 && voltage <= 14)
            power = .8;
        else if(voltage > 14)
            power = .65;
        shoot(power, 350);
        sleep(750);
        double angle50 = getGyroYaw();
        turnRightWithPID(50, .006, .000045, 0.0);
        sleep(500);
        angle50 -= 50;
        moveForwardsToWhiteLine(4000, angle50);
        pushFrontRed();
    }














//    telemetry.addData("init", "test1");
//        telemetry.update();
//        long beforeTime = System.currentTimeMillis();
//        double beforeAngle = getGyroYaw();
//        while(System.currentTimeMillis() - beforeTime < 15000) {
//            telemetry.addData("angle", Math.abs(getGyroYaw() - beforeAngle));
//            telemetry.update();
//        }
//        waitForStart();
//        moveBackwardPID(2500);
//
//        //correct(beforeAngle);
//        moveBackwardsToWhiteLine(300, .175, perpen);
//        beaconValue();
//        moveForwardPID(2300);
//        moveForwardsToWhiteLine(600);
//        turnLeftWithPID(15);
//        moveForwardWithEncoders(.25, 1000);
//        //bring down shooter
//        bringDownShooter(.1, 1100);
//        sleep(750);
//        //shoot
//        shoot(.85, 350);
//        moveForwardWithEncoders(.4, 3000);
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
