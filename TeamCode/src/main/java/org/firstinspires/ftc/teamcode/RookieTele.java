package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by sopa on 9/25/16.
 */
@TeleOp(name = "RookieTele", group = "Teleop")
public class RookieTele extends OpMode
{
    DcMotor FR;
    DcMotor FL;
    @Override
    public void init()
    {
        FR = hardwareMap.dcMotor.get("right motor");
        FL = hardwareMap.dcMotor.get("FL");
        FR.setPower(0);
        FL.setPower(0);
    }
    @Override
    public void loop()
    {
        if(Math.abs(gamepad1.right_stick_y) > 0.5)
        {
            FR.setPower(0);
        }
    }
}
