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
    DcMotor ShooterF;
    DcMotor ShooterB;
    DcMotor ManIn;
    DcMotor ManLift;
    boolean shootfull;
    boolean shootpartial;
    boolean harvest;
    int direction = 1;
    int liftPosO;
    int liftPosCurrent;
    long shoottime;
    long harvesttime;
    long currentTime;
    long lastTime;
    boolean halfspeed = false;
    final double HALFSPEED = .25;
    final double FULLSPEED = 1;
    final long DURATION = 2000000000;
    final int UPDISTANCE = 30;
    double speed = 0;
    boolean liftUp;
    @Override
    public void init()
    {
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        ShooterB = hardwareMap.dcMotor.get("B");
        ShooterF = hardwareMap.dcMotor.get("F");
        ManIn = hardwareMap.dcMotor.get("ManIn");
        ManLift = hardwareMap.dcMotor.get("ManLift");
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        ShooterB.setPower(0);
        ShooterF.setPower(0);
        ManLift.setPower(0);
        ManIn.setPower(0);
        //Shooter.setPower(0);
        ManLift.setPower(0);
        ManIn.setPower(0);
        ManLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //ManLift.setMode(DcMotor.RunMode.RESET_ENCODERS);
        liftPosO = 0;
        liftPosCurrent = 0;
        shootfull = false;
        shootpartial = false;
        harvest = false;
	    liftUp = false;
        shoottime = 0;
        harvesttime = 0;
        currentTime = 0;
        lastTime = 0;
        liftUp = false;
    }

    @Override
    public void loop() {

        //Tank Drive
        if (Math.abs(gamepad1.left_stick_y) > .1)
        {
            FL.setPower(gamepad1.left_stick_y * direction * -1 * speed);
            BL.setPower(gamepad1.left_stick_y * direction * -1 * speed);
        }
        else
        {
            FL.setPower(0);
            BL.setPower(0);
        }
        if (Math.abs(gamepad1.right_stick_y) > .1)
        {
            FR.setPower(gamepad1.right_stick_y * direction * speed);
            BR.setPower(gamepad1.right_stick_y * direction * speed);
        }
        else
        {
            FR.setPower(0);
            BR.setPower(0);
        }
	//HalfSpeed Macro
        if (gamepad1.a)
        {
            currentTime = System.nanoTime();
            if(currentTime > lastTime + DURATION)
            {
                if(halfspeed) {
                    halfspeed = false;
                }
                else
                    halfspeed = true;
            }
            if(halfspeed)
                speed = HALFSPEED;
            else
                speed = FULLSPEED;
            lastTime = System.nanoTime();
        }
        //Reverse Macro
        if(gamepad1.y)
        {
            currentTime = System.nanoTime();
            if(currentTime > lastTime + DURATION)
            {
                direction *= -1;
            }
            lastTime = System.nanoTime();
        }

        //Below is harvester & shooter controls for a second controller. For testing reasons, alternate 1 control
        //program follows
        //Shooter Control - Need to test for various to configure various shooting postions
        if (gamepad2.a)
        {
            ShooterF.setPower(1);
            ShooterB.setPower(-1);
        }
        else if (gamepad2.b)
        {
            ShooterF.setPower(.75);
            ShooterB.setPower(-.75);
        }
        else if (gamepad2.x)
        {
            ShooterF.setPower(.5);
            ShooterB.setPower(-.5);
        }
        else
        {
            ShooterF.setPower(0);
            ShooterB.setPower(0);
        }
        //Manipulator Control
        if(Math.abs(gamepad2.right_stick_y) > .1)
        {
            ManIn.setPower(gamepad2.right_stick_y);
        }
        else
        {
            ManIn.setPower(0);
        }
        if(Math.abs(gamepad2.left_trigger) > .05)
            ManLift.setPower(gamepad2.left_trigger * .25 * -1);

        else if(Math.abs(gamepad2.right_trigger) > .05)
            ManLift.setPower(gamepad2.right_trigger * .25);

        else
            ManLift.setPower(0);
        /**if(gamepad2.y)
        {
            currentTime = System.nanoTime();
            if(currentTime > lastTime + DURATION)
            {
                if(liftUp) liftUp = false;
                else liftUp = true;
            }
            lastTime = System.nanoTime();
        }
        /**if(Math.abs(gamepad2.left_stick_y) > .05)
        {
            ManLift.setPower(-1 * gamepad2.left_stick_y * .1);
        }
        else
            ManLift.setPower(0);
        if(Math.abs(gamepad2.left_trigger) > .05)
            ManLift.setPower(gamepad2.left_trigger * .1);
        else
            ManLift.setPower(0);
        liftPosO = ManLift.getCurrentPosition();
        if(liftUp && Math.abs(ManLift.getCurrentPosition() - liftPosO) < UPDISTANCE)
        {
            ManLift.setPower(-.3);
            telemetry.addData("upPosition", ManLift.getCurrentPosition());
        }
        else if(!liftUp && ManLift.getCurrentPosition() > 0) {
            ManLift.setPower(.3);
        }
        else
        {
            ManLift.setPower(0);
        }
         */
    }
}
/* Shooter toggle controls - Please check this code to make sure it is fine
button y is now a toggle which controls liftUp boolean. If liftUp boolean is true
and the encoder does not read 0 (we are starting up, so the encoder should read 0 at the top)
if(gamepad2.y)
        {
            currentTime = System.nanoTime();
            if(currentTime > lastTime + DURATION)
            {
                if(liftUp) liftUp = false;
		else liftUp = true;
            }
            lastTime = System.nanoTime();
        }
if(liftUp && ManLift.getCurrentPosition() > 0)
{
	ManLift.setPower(-.3);
}
else if(!liftUp && ManLift.getCurrentPosition() != UPDISTANCE)
{
	ManLift.setPower(.3);
}
else
{
	ManLift.setPower(0);
}


*/