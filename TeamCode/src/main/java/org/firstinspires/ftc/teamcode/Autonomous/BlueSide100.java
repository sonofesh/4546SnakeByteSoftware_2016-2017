package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/31/16.
 * Test count:
 * Shoot first, hit beacons sequentially, hit cap ball, park
 * test count: 7 + 12 + 12 + 11 + 4
 */

@Autonomous(name = "BlueKrishna", group = "Autonomous")
public class BlueSide100 extends AutoOpMode {
    public BlueSide100() { super(); }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
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
        turnRightWithPID(42, .006, .000045, 0.0);
        sleep(500);
        moveForwardPID(.00025, .00000003, 0.0, 3800);
        sleep(500);
        correctOneSide(perpendicular, .006, .000015, 0.0, 0.0);
        sleep(500);
        moveBackwardsToWhiteLine(550);
        sleep(500);
        pushBlueBeacon();
        sleep(1000);
        moveForwardPID(2500, perpendicular);
        moveForwardsToWhiteLine(300);
        pushBlueBeacon();
        sleep(1000);
        moveBackwardsWithATiltRight(.6, 4200);
    }
}
