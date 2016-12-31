package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by sopa on 12/31/16.
 */
@Autonomous(name = "TestBeaconPusher", group = "Autonomous")
@Disabled
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
        telemetry.addData("test4", "init");
        waitForStart();
        telemetry.log().add("servoPos-AutoBlue", AutoBeaconB.getPosition());
        telemetry.update();
        sleep(5000);
        telemetry.log().add("servoPos-AutoBlue", AutoBeaconB.getPosition());
        telemetry.update();
        sleep(5000);
    }
}
