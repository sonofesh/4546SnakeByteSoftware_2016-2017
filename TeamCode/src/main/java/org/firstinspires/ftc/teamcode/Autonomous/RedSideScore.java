package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by 4546 Snakebyte on 12/19/16.
 * Test count: 18 + 3 + 7 + 18 +
 * This will essentially be our red side auto, provided the first turn is reversed
 * DELETE WHEN DONE.
 * Shoot first auto, configured for red side
 */
@Autonomous(name = "RedSide70", group = "Autonomous")
public class RedSideScore extends AutoOpMode {
    public RedSideScore() {
        super();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("init", "test8");
        telemetry.update();
        waitForStart();
        double perpendicular = getGyroYaw();
        double angle43 = perpendicular;
        //int movement = 0;
        moveForward(.175, 500);
        //moveForwardPID(500);
        //bring down shooter
        bringDownShooter(.1, 1200);
        sleep(1000);
        //shoot
        double power = .8;
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        if (voltage <= 13.5)
            power = .85;
        else if (voltage <= 13.75 && voltage > 13.5)
            power = .8;
        else if (voltage > 13.75 && voltage <= 13.9)
            power = .75;
        else if (voltage > 13.9 && voltage <= 14)
            power = .68;
        else if (voltage > 14)
            power = .65;
        shoot(power, 350);
        sleep(750);
        turnLeftWithPID(43, .005, .00003, 0.0);
        sleep(500);
        angle43 -= 43;
        moveForwardsToWhiteLine(2850, angle43);
        moveForwardWithEncoders(.15, 150);
        //double p = .0002; double i = .00000015; //double d = 2.0;
        sleep(500);
        turnIntoWhiteLineRed(Math.abs((getGyroYaw() - perpendicular)));
        sleep(500);
        pushFrontRed(perpendicular - 90);
        sleep(500);
        //turnRightWithGyro(.3, 5);
        sleep(500);
        bringDownShooter(-.4, 800);
        moveBackWardWithEncoders(.6, 2800);
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
}
