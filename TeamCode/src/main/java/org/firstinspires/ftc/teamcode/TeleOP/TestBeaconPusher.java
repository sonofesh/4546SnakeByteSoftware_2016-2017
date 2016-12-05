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
    @Override
    public void init()
    {
        Beacon = hardwareMap.servo.get("Beacon");
        telemetry.addData("test4", "init");
        Beacon.setPosition(1);
    }

    @Override
    public void loop()
    {
        if(gamepad1.a)
            Beacon.setPosition(1);
        else if(gamepad1.b)
            Beacon.setPosition(.45);
        else if(gamepad1.x)
            Beacon.setPosition(.4);
        else if (gamepad1.y)
            Beacon.setPosition(.2);
    }
}
