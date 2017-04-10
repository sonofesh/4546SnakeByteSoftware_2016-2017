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
        sleep(1000);
        double power = .88;
        telemetry.addData("init", "test1");
        telemetry.update();
        waitForStart();
        double startLight = colorSensorAverageValues();
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        double parallel = getGyroYaw();
        moveForwardWithEncoders(.2, 500);
        //bring down shooter
//        bringDownShooter(.35, 1100);
//        sleep(750);
//        if (voltage <= 13.5)
//            power = .96;
//        else if (voltage <= 13.75 && voltage > 13.5)
//            power = .93;
//        else if (voltage > 13.75 && voltage <= 13.9)
//            power = .89;
//        else if (voltage > 13.9 && voltage <= 14)
//            power = .87;
//        else if (voltage > 14)
//            power = .82;
//        shoot(power, 380);
//        bringDownShooter(-.45, 400);
        double angle50 = getGyroYaw(); //Shouldn't this be minus change?
        turnLeftWithPID(50, .00375, .0000125, 0.0); //CHANGED 32 to 45 to 42
        sleep(500);
        angle50 -= 50;
        //double p = .004; double i = .000015; //double d = 2.0;
        moveStartToWall(3100, angle50); // removed rang sensor for now.
        turnRightWithPID(15, .003, .000015, 0.0);
        moveToWallRed_Stop(3000, .3, startLight + 5);
        sleep(500);
        resetEncoders();
//        if (onWhiteLine(startLight + 5) == false)
//            moveToSecondLine(2000, .275);
        sleep(500);
        if (onWhiteLine(startLight + 3) == false) {
            sleep(500);
            moveBackToWhiteLine(200, .115, startLight + 3);
        }
        pushRedBeacon();
        resetEncoders();
        //correct(parallel, .04, .00015, 0.0, 0);
//        moveToSecondLine(3000, .25);
        moveToSecondLine(4000, .2, startLight + 4);
        sleep(500);
        if (onWhiteLine(startLight + 3) == false) {
            sleep(500);
            moveBackToWhiteLine(300, -.115, startLight + 3);
        }
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
