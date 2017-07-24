package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by sopa on 9/25/16.
 */
@TeleOp(name = "Teleop", group = "Teleop")
public class RookieTeleOp extends OpMode
{
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    @Override
    public void init()
    {
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }
    @Override
    public void loop()
    {
        if(Math.abs(gamepad1.right_stick_y) > 0.5)
        {
            FR.setPower(gamepad1.right_stick_y);
            BR.setPower(gamepad1.right_stick_y);
        }
        else
        {
            FR.setPower(0);
            BR.setPower(0);
        }
        if(Math.abs(gamepad1.left_stick_y) > 0.5)
        {
            FL.setPower(gamepad1.left_stick_y * -1);
            BL.setPower(gamepad1.left_stick_y * -1);
        }
        else
        {
            FL.setPower(0);
            BL.setPower(0);
        }
    }
}
