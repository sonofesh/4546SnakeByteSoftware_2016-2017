package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

/**
 * Created by sopa on 11/22/16.
 */
@TeleOp(name = "TestBeaconPusher", group = "Teleop")
@Disabled
public class TestBeaconPusher extends OpMode {
    //Servo ManBeaconL;
    //Servo ManBeaconR;
    Servo AutoBeaconB;
    //Servo AutoBeaconR;
    double pos = 0;
    @Override
    public void init()
    {
        //ManBeaconL = hardwareMap.servo.get("ManBeaconL");
        //ManBeaconR = hardwareMap.servo.get("ManBeaconR");
        AutoBeaconB = hardwareMap.servo.get("AutoBeaconL");
        //AutoBeaconR = hardwareMap.servo.get("AutoBeaconR");
        telemetry.addData("test4", "init");
        AutoBeaconB.setPosition(.5);
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            pos += .01;
        }
        else if (gamepad1.b) {
            pos -= .01;
        }
        AutoBeaconB.setPosition(pos);
        telemetry.log().add("ServoPos", AutoBeaconB.getPosition());
        telemetry.update();
        /* telemetry.addData("servoPos-AutoL", AutoBeaconB.getPosition());
        telemetry.update();
        try {
            sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        telemetry.addData("servoPos-AutoL", AutoBeaconB.getPosition());
        telemetry.update();
        telemetry.addData("servoPos-AutoR", AutoBeaconR.getPosition());
        telemetry.update();
        try {
            sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        telemetry.addData("servoPos-AutoR", AutoBeaconR.getPosition());
        telemetry.update();
        telemetry.addData("servoPos-ManL", ManBeaconL.getPosition());
        telemetry.update();
        try {
            sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        telemetry.addData("servoPos-ManL", ManBeaconL.getPosition());
        telemetry.update();
        telemetry.addData("servoPos-ManR", ManBeaconR.getPosition());
        telemetry.update();
        try {
            sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        telemetry.addData("servoPos-ManR", ManBeaconR.getPosition());
        telemetry.update(); */

    }
}
