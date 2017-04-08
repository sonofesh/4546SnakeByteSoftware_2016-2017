package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/31/16.
 * Test count:
 * Shoot first, hit beacons sequentially, hit cap ball, park
 * test count: 9 + 7 + 24 +
 */

@Autonomous(name = "Blue100Worlds", group = "Autonomous")
public class Blue100Worlds extends AutoOpMode {
    public Blue100Worlds() {
        super();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        double power = .88;
        telemetry.addData("init", "test1");
        telemetry.update();
        waitForStart();
        double startLight = colorSensorAverageValues();
        double parallel = getGyroYaw();
        moveForwardWithEncoders(.2, 500);
        //moveForwardPID(500);
        //bring down shooter
        bringDownShooter(.28, 1150);
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
        double angle45 = getGyroYaw();
        bringDownShooter(-.4, 620);
        if(getGyroYaw() + 45 > 360) {
            double firstTurn = Math.abs(360 - getGyroYaw());
            turnRightWithPID(firstTurn, .005, .00003, 0.0);
            turnRightWithPID(45 - firstTurn, .0055, .00006, 0.0);
        }
        else
            turnRightWithPID(45, .00525, .00003, 0.0);
        turnRightWithPID(45, .00525, .00003, 0.0); //CHANGE - Steeper angle
        angle45 += 45;
        moveStartToWall(3000, angle45);
        sleep(750);
        double turn = 160;
        if(getGyroYaw() + 160 > 360) {
            double firstTurn = Math.abs(360 - getGyroYaw());
            turnRightWithPID(firstTurn, .005, .000025, 0.0);
            turnRightWithPID(160 - firstTurn, .0054, .00006, 0.0);
        }
        else
            turnRightWithPID(160, .004, .000045, 0.0);
        resetEncoders();
        moveToWallBlue(2450, .325);
        sleep(500);
        if(onWhiteLine() == false)
            moveBackToWhiteLine(1000, -.15, startLight + 7);
        sleep(500);
        pushBlueBeacon();
        sleep(1000);
        resetEncoders();
        //correct(parallel, .04, .00015, 0.0, 0);
//        moveForwardPID(2500, parallel);
//        moveForwardsToWhiteLine(300, parallel);
//        moveToSecondLine(4000, -.3);
        sleep(500);
        if(onWhiteLine() == false)
            moveBackToWhiteLine(600, .125, startLight + 6);
        sleep(500);
        pushBlueBeacon();
        sleep(500);
        moveBackWardWithEncoders(.4, 800);
        turnLeftWithGyro(.4, 90);
        sleep(500);
        moveBackWardWithEncoders(.5, 3000);
    }
}
