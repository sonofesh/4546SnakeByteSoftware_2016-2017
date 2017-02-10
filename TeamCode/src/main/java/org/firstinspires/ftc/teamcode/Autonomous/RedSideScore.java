package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by 4546 Snakebyte on 12/19/16.
 * Test count: 18 + 3 + 7 + 18 + 8 + 5 + 11 + 11 + 14
 * This will essentially be our red side auto, provided the first turn is reversed
 * DELETE WHEN DONE.
 * Shoot first auto, configured for red side
 */
@Autonomous(name = "BloodDiamonds", group = "Autonomous")
public class RedSideScore extends AutoOpMode {
    public RedSideScore() {
        super();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        double power = .88;
        telemetry.addData("init", "test13");
        telemetry.update();
        waitForStart();
        double perpendicular = getGyroYaw();
        moveForwardWithEncoders(.16, 500);
        //moveForwardPID(500);
        //bring down shooter
        bringDownShooter(.1, 1150);
        sleep(1000);
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        if (voltage <= 13.5)
            power = .97;
        else if (voltage <= 13.75 && voltage > 13.5)
            power = .94;
        else if (voltage > 13.75 && voltage <= 13.9)
            power = .9;
        else if (voltage > 13.9 && voltage <= 14)
            power = .88;
        else if (voltage > 14)
            power = .83;

        shoot(power, 360);
        double change = getGyroYaw() - perpendicular;
        double angle36 = getGyroYaw();
        turnLeftWithPID(36 + change, .004, .00002, 0.0);
        sleep(500);
        angle36 -= 36;
        //double p = .004; double i = .000015; //double d = 2.0;
        moveForwardPID(4200, angle36);
//        correctOneSideLeft(perpendicular, .0042, .000012, 0, 20);
        moveToWall(2900, .25);
        sleep(500);
        moveToSecondLine(1675, .2);
        sleep(500);
        moveBackToWhiteLine(600, -.1);
        sleep(500);
        pushRedBeacon();
        sleep(500);
        //correct(perpendicular, .04, .00015, 0.0, 0);
//        moveToSecondLine(3000, .25);
        moveToSecondLine(2950, .25);
        sleep(500);
        resetCount();
        moveBackToWhiteLine(600, -.115);
        sleep(500);
        pushRedBeacon();
        sleep(500);
        turnRightWithGyroOneSide(.45, 80);
        sleep(500);
        moveForwardWithEncoders(.5, 2000);
        sleep(500);
        turnLeftWithGyro(-.75, 45);

    }

    /*
     correct(perpendicular, .04, .00015, 0.0, 0);
        moveForwardPID(2500, perpendicular);
        sleep(1000);
        movement = 0;
        moveForwardsToWhiteLine(300 ,perpendicular);
        pushRedBeacon(perpendicular);
        sleep(1000);
        moveBackwardsWithATiltRight(.4, 4200);
     */


//    initialize();
//    telemetry.addData("init", "test1");
//    telemetry.update();
//    waitForStart();
//    double perpendicular = getGyroYaw();
//    double angle42 = perpendicular;
//    //int movement = 0;
//    moveForward(.15, 500);
//    //moveForwardPID(500);
//    //bring down shooter
//    bringDownShooter(.1, 1150);
//    sleep(1000);
//    //shoot
//    double power = .8;
//    double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
//    if (voltage <= 13.5)
//    power = .95;
//    else if (voltage <= 13.75 && voltage > 13.5)
//    power = .925;
//    else if (voltage > 13.75 && voltage <= 13.9)
//    power = .875;
//    else if (voltage > 13.9 && voltage <= 14)
//    power = .85;
//    else if (voltage > 14)
//    power = .8;
//    shoot(power, 360);
//    sleep(750);
//    turnLeftWithPID(42, .005, .00003, 0.0);
//    sleep(500);
//    angle42 -= 42;
//    moveForwardsToWhiteLine(2850, angle42);
//    moveForwardWithEncoders(.15, 150);
//    //double p = .0002; double i = .00000015; //double d = 2.0;
//    sleep(500);
//    turnIntoWhiteLineRed(Math.abs((getGyroYaw() - perpendicular)));
//    sleep(500);
//    pushFrontRed(perpendicular - 90);
//    sleep(500);
//    //turnRightWithGyro(.3, 5);
//    sleep(500);
//    bringDownShooter(-.4, 800);
//    moveBackWardWithEncoders(.6, 2700);
//    turnRightWithGyro(.3, 30);
}
