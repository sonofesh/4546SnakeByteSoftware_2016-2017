package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import javax.xml.datatype.Duration;

/**
 * Created by sopa on 11/30/16.
 */
public class GetShooting extends OpMode {
    DcMotor FR;
    DcMotor BR;
    DcMotor FL;
    DcMotor BL;
    DcMotor ShooterB;
    DcMotor ShooterF;
    DcMotor ManLift;
    DcMotor ManIn;
    Servo Beacon;
    Servo Stopper;
    Servo Delayer;
    //average encoder value
    int beforeALV = 0;
    int beforeMLV = 0;
    double beforeAngle = 2;
    final double blackAVC = 2;
    final double whiteACV = 27;
    final long DURATION = 2000;
    double orginalYawAngle = 0;
    long currTime  = 0;
    @Override
    public void init() {
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        ShooterB = hardwareMap.dcMotor.get("B");
        ShooterF = hardwareMap.dcMotor.get("F");
        ManLift = hardwareMap.dcMotor.get("ManLift");
        ManIn = hardwareMap.dcMotor.get("ManIn");
        Beacon = hardwareMap.servo.get("Beacon");
        Delayer = hardwareMap.servo.get("Delayer");
        Stopper = hardwareMap.servo.get("Stopper");
        ManIn = hardwareMap.dcMotor.get("ManIn");
        Beacon.setPosition(.6);
        Delayer.setPosition(.9);
        Stopper.setPosition(0);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ManLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ManLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("gyro", "initalizing");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        currTime = System.currentTimeMillis();
        if(gamepad1.a){
            if(Math.abs(currTime - System.currentTimeMillis()) > DURATION){

            }

        }

    }
}
