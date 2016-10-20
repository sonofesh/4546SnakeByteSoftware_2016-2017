package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * VHS ROBOTICS 4546
 * 10/17/16
 */

//TeleOp Version A
@TeleOp(name = "TeleOpA", group = "Teleop")
public class TeleOpA extends OpMode{
    @Override

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
        havest = false;
        shoottime = 0;
        harvesttime =0;
        currenttime = 0;
    }

    @Override
    public void loop() {

        //Tank Drive
        if (gamepad1.left_stick_y > .1) {
            FL.setPower(gamepad1.left_stick_y);
            BL.setPower(gamepad1.left_stick_y);
        } else {
            FL.setPower(0);
            BL.setPower(0);
        }
        if (gamepad1.right_stick_y > .1) {
            FR.setPower(-gamepad1.right_stick_y);
            FL.setPower(-gamepad1.right_stick_y);
        } else {
            FR.setPower(0);
            BR.setPower(0);
        }

        //Update Current Time
        currenttime = System.nanotime();

        //Shooter Control - Toggle A for Full Power, Toggle B for 3/4 Power
        if (gamepad1.a) {
            if (shootfull == false) {
                shootpartial = false;
                shootfull = true;
                time = System.nanoTime();
            } else if ((currenttime - shoottime)/1000000 == 500) {
                shootfull = false;
            }
        }
        if (gamepad1.b) {
            if (shootpartial = false) {
                shootfull = false;
                shootpartial = true;
                time = System.nanoTime();
            } else if ((currenttime - shoottime)/1000000 == 500) {
                shootpartial = false;
            }
        }
        if (shootfull) Shooter.setPower(1);
        else if (shootpartial) Shooter.setPower(.75);
        else Shooter.setPower(0);

        //Harvester Control - Toggle X
        if (gamepad1.a) {
            if (harvest == false) {
                harvest == true;
                harvesttime = System.nanoTime();
            } else if ((currenttime - harvesttime)/1000000 == 500) {
                harvest = false;
        }
    }

        if (harvest) {
            HarvesterL.setPower(1);
            HarvesterR.setPower(1);
        } else {
            HarvesterL.setPower(0);
            HarvesterR.setPower(0);
        }
    }
}