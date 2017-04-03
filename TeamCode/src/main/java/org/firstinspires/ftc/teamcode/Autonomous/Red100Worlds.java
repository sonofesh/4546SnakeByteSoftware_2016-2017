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
@Autonomous(name = "Red100Worlds", group = "Autonomous")
public class Red100Worlds extends AutoOpMode {
    public Red100Worlds() {
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
        double parallel = getGyroYaw();
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
        double change = getGyroYaw() - parallel;
        double angle45 = getGyroYaw() + change; //Shouldn't this be minus change?
        turnLeftWithPID(45, .003, .0000175, 0.0); //CHANGED from 32 to 45 - steeper angle
        sleep(500);
        angle45 -= 45;
        //double p = .004; douzble i = .000015; //double d = 2.0;
        moveStartToWall(3000, angle45, 50, parallel); //New method - moves till range sensor detects close or until encoder cap
        correctOneSideLeft(parallel, .0042, .000012, 0, 30);
        turnRightWithGyroOneSide(.4,40); //Turn into wall
        moveToWallRed(2000, .3);
        sleep(500);
        resetEncoders();
        if (onWhiteLine(startLight + 5) == false)
            moveToSecondLine(2000, .275);
        sleep(500);
        if (onWhiteLine(startLight + 5) == false)
            moveBackToWhiteLine(650, -.14, startLight + 5);
        pushRedBeacon();
        resetEncoders();
        //correct(parallel, .04, .00015, 0.0, 0);
//        moveToSecondLine(3000, .25);
        moveToSecondLine(4000, .3);
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
}
