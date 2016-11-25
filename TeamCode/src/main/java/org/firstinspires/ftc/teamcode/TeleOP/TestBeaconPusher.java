package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by sopa on 11/22/16.
 */
@TeleOp(name = "TestBeaconPusher", group = "Teleop")
public class TestBeaconPusher extends OpMode {
    Servo Beacon;
    Servo Stopper;
    Servo Delayer;
    @Override
    public void init()
    {
        Beacon = hardwareMap.servo.get("Beacon");
        Stopper = hardwareMap.servo.get("Stopper");
        Delayer = hardwareMap.servo.get("Delayer");
        telemetry.addData("test11", "init");
        Beacon.setPosition(.6);
        Stopper.setPosition(.5);
        Delayer.setPosition(.7);
    }

    @Override
    public void loop()
    {
        if(gamepad1.a)
            Beacon.setPosition(.8);
        else
            Beacon.setPosition(.6);
        if(gamepad1.x)
            Delayer.setPosition(.7);
        else
            Stopper.setPosition(.2);
    }
}
