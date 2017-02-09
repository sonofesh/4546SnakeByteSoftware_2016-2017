package org.firstinspires.ftc.teamcode.Autonomous.OldAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/31/16.
 * Test count:
 * Shoot first, hit beacons sequentially, hit cap ball, park
 * test count: 0
 */

@Autonomous(name = "RedSide100", group = "Autonomous")
@Disabled
public class RedSide100 extends AutoOpMode{
        public RedSide100() { super(); }

    @Override
    public void runOpMode() throws InterruptedException {
//        initialize();
//        telemetry.addData("init", "test0");
//        telemetry.update();
//        waitForStart();
//        double perpendicular = getGyroYaw();
//        double angle43 = perpendicular;
//        //int movement = 0;
//        moveForward(.175, 500);
//        //moveForwardPID(500);
//        //bring down shooter
//        bringDownShooter(.1, 1150);
//        sleep(1000);
//        //shoot
//        double power = .8;
//        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
//        if (voltage <= 13.5)
//            power = .85;
//        else if (voltage <= 13.75 && voltage > 13.5)
//            power = .8;
//        else if (voltage > 13.75 && voltage <= 13.9)
//            power = .75;
//        else if (voltage > 13.9 && voltage <= 14)
//            power = .68;
//        else if (voltage > 14)
//            power = .65;
//        shoot(power, 350);
//        sleep(750);
//        turnLeftWithPID(43, .005, .00003, 0.0);
//        sleep(500);
//        angle43 -= 43;
//        moveForwardsToWhiteLine(2850, angle43);
//        moveForwardWithEncoders(.15, 150);
//        //double p = .0002; double i = .00000015; //double d = 2.0;
//        sleep(500);
//        turnIntoWhiteLineRed(Math.abs((getGyroYaw() - perpendicular)));
//        sleep(500);
//        pushFrontRed(perpendicular - 90);
//        sleep(500);
//        //
//        moveBackWardWithEncoders(.4, 400);
//        turnRightWithPID(Math.abs(getGyroYaw() - perpendicular));
//        sleep(500);
//        moveForwardsToWhiteLine(2500, perpendicular);
//        sleep(500);
//        turnIntoWhiteLine(Math.abs(getGyroYaw() - perpendicular));
//        sleep(500);
//        pushSecondRed(perpendicular + 90);
    }
}
