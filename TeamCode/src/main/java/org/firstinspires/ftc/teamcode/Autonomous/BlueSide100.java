package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/31/16.
 * Test count:
 * Shoot first, hit beacons sequentially, hit cap ball, park
 * test count: 9 + 7
 */

@Autonomous(name = "BlueSide90", group = "Autonomous")
public class BlueSide100 extends AutoOpMode {
    public BlueSide100() {
        super();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        double power = .88;
        telemetry.addData("init", "test7");
        telemetry.update();
        waitForStart();
        double perpendicular = getGyroYaw();
        moveForwardWithEncoders(.16, 600);
        //moveForwardPID(500);
        //bring down shooter
        bringDownShooter(.3, 1150);
        sleep(1000);
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        if (voltage <= 13.5)
            power = .95;
        else if (voltage <= 13.75 && voltage > 13.5)
            power = .92;
        else if (voltage > 13.75 && voltage <= 13.9)
            power = .88;
        else if (voltage > 13.9 && voltage <= 14)
            power = .86;
        else if (voltage > 14)
            power = .80;
        shoot(power, 360);
        double angle38 = getGyroYaw();
        bringDownShooter(-.4, 600);
        turnRightWithPID(38, .005, .000025, 0.0);
        angle38 += 38;
        moveForwardPID(3200, angle38);
        sleep(500);
        double turn = 160;
        if(getGyroYaw() + 160 > 360) {
            double firstTurn = Math.abs(360 - getGyroYaw());
            turnRightWithPID(firstTurn, .005, .00004, 0.0);
            turnRightWithPID(165 - firstTurn, .0055, .00005, 0.0);
        }
        else
            turnRightWithPID(165, .004, .00004, 0.0);
//        correctOneSideRight(perpendicular, .004, .000015, 0, 0);
//        sleep(500);
        moveBackWardWithEncoders(.2, 450);
        resetEncoders();
        moveToWallBlue(2050, .325);
        sleep(500);
        if(onWhiteLine() == false)
            moveBackToWhiteLine(600, -.14, 12);
        sleep(500);
        pushBlueBeacon();
        sleep(1000);
        //correct(perpendicular, .04, .00015, 0.0, 0);
//        moveForwardPID(2500, perpendicular);
//        moveForwardsToWhiteLine(300, perpendicular);
        moveToSecondLine(3600, -.3);
        sleep(500);
        if(onWhiteLine() == false)
            moveBackToWhiteLine(700, .14, 8);
        sleep(500);
        pushBlueBeacon();
        sleep(500);
//        moveBackWardWithEncoders(.4, 800);
//        turnLeftWithGyro(.4, 90);
//        sleep(500);
//        moveBackWardWithEncoders(.5, 3000);
    }
}
    /*initialize();
        double power = .8;
        telemetry.addData("init", "test1");
        telemetry.update();
        waitForStart();
        double perpendicular = getGyroYaw();
        int movement = 0;
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
        double angle43 = getGyroYaw();
        turnRightWithPID(43, .006, .000045, 0.0);
        sleep(500);
        angle43 -= 43;
        moveForwardPID(4000, angle43);
//        sleep(500);
//        correctOneSide(perpendicular, .006, .000015, 0.0, 0.0);
//        sleep(500);
//        moveBackwardsToWhiteLine(1200, .175, perpendicular);
//        sleep(500);
//        pushBlueBeacon(perpendicular);
//        sleep(1000);
//        correct(perpendicular, .04, .00015, 0.0, 0);
//        moveForwardPID(2500, perpendicular);
//        moveForwardsToWhiteLine(300, perpendicular);
//        pushBlueBeacon(perpendicular);
}
*/

//        initialize();
//        double power = .8;
//        telemetry.addData("init", "test0");
//        telemetry.update();
//        waitForStart();
//        double perpendicular = getGyroYaw();
//        moveForward(.16, 500);
//        //moveForwardPID(500);
//        //bring down shooter
//        bringDownShooter(.1, 1150);
//        sleep(1000);
//        //shoot
//        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
//        if (voltage <= 13.5)
//            power = .825;
//        else if (voltage <= 13.75 && voltage > 13.5)
//            power = .78;
//        else if (voltage > 13.75 && voltage <= 13.9)
//            power = .725;
//        else if (voltage > 13.9 && voltage <= 14)
//            power = .66;
//        else if (voltage > 14)
//            power = .635;
//        shoot(power, 350);
//        sleep(750);
//        double angle43 = perpendicular;
//        //double p = .004; double i = .000015;
//        turnRightWithPID(43, .006, .00004, 0.0);
//        sleep(500);
//        angle43 += 43;
//        moveForwardsToWhiteLine(2850, angle43);
//        moveForwardWithEncoders(.15, 20);
//        //double p = .004; double i = .000015;
//        sleep(500);
//        turnIntoWhiteLine(Math.abs(getGyroYaw() - perpendicular) + 5);
//        sleep(500);
//        pushFrontBlue(perpendicular + 90);
//        sleep(500);
//        moveBackWardWithEncoders(.4, 400);
//        turnLeftWithPID(Math.abs(getGyroYaw() - perpendicular));
//        sleep(500);
//        moveForwardsToWhiteLine(2500, perpendicular);
//        sleep(500);
//        turnIntoWhiteLine(Math.abs(getGyroYaw() - perpendicular));
//        sleep(500);
//        pushSecondBlue(perpendicular + 90);
