package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/31/16.
 * Test count:
 * Shoot first, hit beacons sequentially, hit cap ball, park
 * test count: 9
 */

@Autonomous(name = "BlueSide100", group = "Autonomous")
public class BlueSide100 extends AutoOpMode {
    public BlueSide100() {
        super();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        double power = .88;
        telemetry.addData("init", "test1");
        telemetry.update();
        waitForStart();
        double perpendicular = getGyroYaw();
        moveForwardWithEncoders(.16, 500);
        //moveForwardPID(500);
        //bring down shooter
        bringDownShooter(.3, 1150);
        sleep(1000);
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
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
        double angle36 = getGyroYaw();
        bringDownShooter(-.4, 500);
        turnRightWithPID(36, .005, .000025, 0.0);
        angle36 += 36;
        moveForwardPID(3200, angle36);
        sleep(500);
        turnRightWithPID(170, .0055, .000045, 0.0);
//        correctOneSideRight(perpendicular, .004, .000015, 0, 0);
//        sleep(500);
        moveBackWardWithEncoders(.2, 300);
        moveToWallBlue(2200, .25);
        sleep(500);
        moveToSecondLine(900, -.2);
        sleep(500);
        moveBackToWhiteLine(800, .16);
        sleep(500);
        pushBlueBeacon();
        sleep(1000);
        //correct(perpendicular, .04, .00015, 0.0, 0);
//        moveForwardPID(2500, perpendicular);
//        moveForwardsToWhiteLine(300, perpendicular);
        moveToSecondLine(3000, -.3);
        sleep(500);
        moveBackToWhiteLine(600, .12);
        sleep(500);
        pushBlueBeacon();
        sleep(500);
        moveBackWardWithEncoders(.4, 800);
        turnLeftWithGyro(.4, 90);
        sleep(500);
        moveForwardWithEncoders(.5, 3000);
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
