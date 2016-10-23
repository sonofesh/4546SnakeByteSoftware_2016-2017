package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * VHS ROBOTICS 4546
 * 10/17/16
 */

//TeleOp Version A
@TeleOp(name = "TeleOpA", group = "Teleop")
public class TeleOpA extends OpMode
{
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor Shooter;
    DcMotor HarvesterR;
    DcMotor HarvesterL;
    boolean shootfull;
    boolean shootpartial;
    boolean harvest;
    long shoottime;
    long harvesttime;
    long currenttime;

    @Override
    public void init() {
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        Shooter = hardwareMap.dcMotor.get("Shooter");
        HarvesterR = hardwareMap.dcMotor.get("HarvesterR");
        HarvesterL = hardwareMap.dcMotor.get("HarvesterL");
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        Shooter.setPower(0);
        HarvesterL.setPower(0);
        HarvesterR.setPower(0);
        shootfull = false;
        shootpartial = false;
        harvest = false;
        shoottime = 0;
        harvesttime =0;
        currenttime = 0;
    }

    @Override
    public void loop() {

        //Tank Drive
        if (gamepad1.left_stick_y > .1)
        {
            FL.setPower(gamepad1.left_stick_y);
            BL.setPower(gamepad1.left_stick_y);
        }
        else
        {
            FL.setPower(0);
            BL.setPower(0);
        }
        if (gamepad1.right_stick_y > .1)
        {
            FR.setPower(-gamepad1.right_stick_y);
            FL.setPower(-gamepad1.right_stick_y);
        }
        else
        {
            FR.setPower(0);
            BR.setPower(0);
        }

        //Update Current Time
        //currenttime = System.nanoTime(); You don't need this for the shooter

        //Shooter Control - Toggle A for Full Power, Toggle B for 3/4 Power
        if (gamepad2.a)
            Shooter.setPower(1);
        else
            Shooter.setPower(0);
        if (gamepad2.b)
           Shooter.setPower(.75);
        else
            Shooter.setPower(0);
        if (gamepad2.x)
            Shooter.setPower(.5);
        else
            Shooter.setPower(0);

        //Manipulator Control
        if(Math.abs(gamepad2.right_stick_y) > .1)
        {
            HarvesterR.setPower(gamepad2.right_stick_y);
            HarvesterL.setPower(gamepad2.right_stick_y * -1);
        }
        else
        {
            HarvesterR.setPower(0);
            HarvesterL.setPower(0);
        }
    }
}