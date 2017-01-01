package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by sopa on 12/31/16.
 * Test: 4
 */
@Autonomous(name = "TestBeaconPusher", group = "Autonomous")
public class TestBeaconPusher extends LinearOpMode {
    Servo ManBeaconL;
    Servo ManBeaconR;
    Servo AutoBeaconB;
    Servo AutoBeaconR;
    @Override
    public void runOpMode() throws InterruptedException {
        ManBeaconL = hardwareMap.servo.get("ManBeaconL");
        ManBeaconR = hardwareMap.servo.get("ManBeaconR");
        AutoBeaconB = hardwareMap.servo.get("AutoBeaconL");
        AutoBeaconR = hardwareMap.servo.get("AutoBeaconR");
        ManBeaconL.setPosition(.2);
        ManBeaconR.setPosition(.8);
        AutoBeaconB.setPosition(0);
        telemetry.addData("test1", "init");
        telemetry.update();
        waitForStart();
        AutoBeaconB.setPosition(1);
        Thread.sleep(1000);
    }
}
