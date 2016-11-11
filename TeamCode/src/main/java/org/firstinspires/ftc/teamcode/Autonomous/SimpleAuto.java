package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by sopa on 11/9/16.
 */
@Autonomous(name = "SimpleAutoRed", group = "Autonomous")
public class SimpleAuto extends LinearOpMode
{
    DcMotor FR;
    DcMotor BR;
    DcMotor FL;
    DcMotor BL;
    DcMotor ShooterB;
    DcMotor ShooterF;
    DcMotor ManLift;
    DcMotor ManIn;
    int standardBRV = 0;
    int standardBLV = 0;
    int BRV = 0;
    int BLV = 0;
    int avg = 0;

    public void moveForward(double power)
    {
        FR.setPower(power);
        BR.setPower(power);
        FL.setPower(-power);
        BL.setPower(-power);
    }
    public void moveBackward(double power)
    {
        moveForward(-power);
    }
    public void turnRight(double power)
    {
        FR.setPower(-power);
        BR.setPower(-power);
        FL.setPower(-power);
        BL.setPower(-power);
    }
    public void bringDownShooter(double power)
    {
        int beforePos = ManLift.getCurrentPosition();
        telemetry.addData("ManLift", ManLift.getCurrentPosition());
        while (Math.abs(ManLift.getCurrentPosition() - beforePos) < 20)
        {
            ManLift.setPower(.2);
        }
        telemetry.addData("ManLift", ManLift.getCurrentPosition());
    }
    public void shoot(double power) throws InterruptedException {
        ShooterF.setPower(power);
        ShooterB.setPower(-power);
        int beforePos = ManLift.getCurrentPosition();
        while(Math.abs(ManLift.getCurrentPosition() - beforePos) < 20)
        {
            ManLift.setPower(.2);
        }
        wait(1000);
        ShooterF.setPower(1);
        ShooterB.setPower(-1);
    }
    public void turnLeft(double power) throws InterruptedException
    {
        turnRight(-power);
    }
    public int getAvg() throws InterruptedException
    {
        BRV = Math.abs(BR.getCurrentPosition());
        BLV = Math.abs(BL.getCurrentPosition());
        avg = Math.abs((BRV + BLV)/2);
        return avg;
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        long startTime = System.currentTimeMillis();
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        ShooterB = hardwareMap.dcMotor.get("B");
        ShooterF = hardwareMap.dcMotor.get("F");
        ManLift = hardwareMap.dcMotor.get("ManLift");
        ManIn = hardwareMap.dcMotor.get("ManIn");
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ManLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        ManLift.setMode(DcMotor.RunMode.RESET_ENCODERS);
        bringDownShooter(.2);
        while(getAvg() < 100)
        {
            moveForward(.2);
        }
        shoot(1);
        long currTime = System.currentTimeMillis();
        if(Math.abs(currTime - startTime) > 15000)
            moveForward(200);
    }
}
