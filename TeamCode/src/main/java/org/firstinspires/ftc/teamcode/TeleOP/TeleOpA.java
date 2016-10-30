package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import javax.xml.datatype.Duration;

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
    //DcMotor Shooter;
    DcMotor ManIn;
    DcMotor ManLift;
    boolean shootfull;
    boolean shootpartial;
    boolean harvest;
    int direction = 1;
    long shoottime;
    long harvesttime;
    long currentTime;
    long lastTime;
    final long DURATION = 2000000000;
    @Override
    public void init() {
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        //Shooter = hardwareMap.dcMotor.get("Shooter");
        ManIn = hardwareMap.dcMotor.get("ManIn");
        ManLift = hardwareMap.dcMotor.get("ManLift");
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        //Shooter.setPower(0);
        ManLift.setPower(0);
        ManIn.setPower(0);
        shootfull = false;
        shootpartial = false;
        harvest = false;
        shoottime = 0;
        harvesttime = 0;
        currentTime = 0;
        lastTime = 0;
    }

    @Override
    public void loop() {

        //Tank Drive
        if (Math.abs(gamepad1.left_stick_y) > .1)
        {
            FL.setPower(gamepad1.left_stick_y * direction * -1);
            BL.setPower(gamepad1.left_stick_y * direction * -1);
        }
        else
        {
            FL.setPower(0);
            BL.setPower(0);
        }
        if (Math.abs(gamepad1.right_stick_y) > .1)
        {
            FR.setPower(gamepad1.right_stick_y * direction);
            BR.setPower(gamepad1.right_stick_y * direction);
        }
        else
        {
            FR.setPower(0);
            BR.setPower(0);
        }

        if(gamepad1.a)
        {
            currentTime = System.nanoTime();
            if(currentTime > lastTime + DURATION)
            {
                direction *= -1;
            }
            lastTime = System.nanoTime();
        }


        //Shooter Control - Need to test for various to configure various shooting postions
        /**if (gamepad2.a)
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
         */
        //Manipulator Control
        /*if(Math.abs(gamepad2.right_stick_y) > .1)
        {
            ManIn.setPower(gamepad2.right_stick_y);
            ManLift.setPower(gamepad2.right_stick_y * -1);
        }
        else
        {
            ManIn.setPower(0);
            ManLift.setPower(0);
        }
        */
        if(gamepad1.a)
        {
            ManIn.setPower(.5);
        }
        else
        {
            ManIn.setPower(0);
        }
        if(gamepad1.right_trigger > .5)
        {
            ManLift.setPower(gamepad1.right_trigger * .5);
        }
        else if (gamepad1.left_trigger>.5)
        {
            ManLift.setPower(-gamepad1.left_trigger * .5);
        }
        else
        {
            ManLift.setPower(0);
        }
    }
}