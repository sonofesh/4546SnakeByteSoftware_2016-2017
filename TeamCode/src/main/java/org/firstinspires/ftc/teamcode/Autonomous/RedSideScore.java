package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by 4546 Snakebyte on 12/19/16.
 * Test count: 18 + 3 + 7 + 18 + 8 + 5 + 11 + 11 + 14 + 14 + 2
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
        sleep(2000);
        double power = .88;
        telemetry.addData("init", "test1");
        telemetry.update();
        waitForStart();
        double startLight = colorSensorAverageValues();
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        double perpendicular = getGyroYaw();
        moveForwardWithEncoders(.2, 500);
        //moveForwardPID(500);
        //bring down shooter
        bringDownShooter(.35, 1100);
        sleep(750);
        if (voltage <= 13.5)
            power = .96;
        else if (voltage <= 13.75 && voltage > 13.5)
            power = .93;
        else if (voltage > 13.75 && voltage <= 13.9)
            power = .89;
        else if (voltage > 13.9 && voltage <= 14)
            power = .87;
        else if (voltage > 14)
            power = .82;
        shoot(power, 380);
        bringDownShooter(-.45, 400);
        double change = getGyroYaw() - perpendicular;
        double angle32 = getGyroYaw() + change;
        turnLeftWithPID(32, .003, .0000175, 0.0);
        sleep(500);
        angle32 -= 32;
        //double p = .004; douzble i = .000015; //double d = 2.0;
        moveForwardPID(4450, angle32);
        correctOneSideLeft(perpendicular, .0042, .000012, 0, 30);
        moveToWallRed(2000, .3);
        sleep(500);
        resetEncoders();
        if(onWhiteLine(startLight + 5) == false)
            moveToSecondLine(2000, .275, 6);
        sleep(500);
        if(onWhiteLine(startLight + 5) == false)
            moveBackToWhiteLine(650, -.14, startLight + 5);
        pushRedBeacon();
        resetEncoders();
        //correct(perpendicular, .04, .00015, 0.0, 0);
//        moveToSecondLine(3000, .25);
        moveToSecondLine(4000, .3, 6);
        sleep(500);
        moveBackToWhiteLine(850, -.14, startLight + 8);
        sleep(500);
        pushRedBeacon();
        sleep(500);
        moveForwardWithEncoders(.5, 500);
        turnRightWithGyroOneSide(.6, 80);
        sleep(500);
        moveForwardWithEncoders(.5, 1700);
        sleep(500);
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
