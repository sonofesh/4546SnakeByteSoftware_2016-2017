/** CONTROLS
 *      Controller 1 - Drive
 *          Right Stick Y Axis : Right Wheel Power          Left Stick Y Axis : Left Wheel Power
 *          Y : Reverse Drive Toggle                        A : Half Speed Toggle
 *
 *      Controller 2 - Scoring
 *          Right Stick Y Axis : Manipulator High Power     Left Stick Y Axis : Manipulator Low Power
 *          Right Trigger : Ramp Down                       Left Trigger : Ramp Uo
 *          Right Bumper : Shooter High Power               Left Bumper : Shooter Low Power
 *          Hold X : Beacon Pusher Left                     Hold B : Beacon Pusher Right
 *          Y : Deploy Stopper Toggle                       A : Deploy Lift Gate
 *
 */



package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import javax.xml.datatype.Duration;

/**
* Created by 4546 on 12/22/16
 * The major change with this TeleOp is that it contains motor scaling for more accurate and precise driving
 */

//TeleOp Version A
@TeleOp(name = "TeleOpA", group = "TeleOp")
public class TeleOpA extends OpMode {
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor ShooterF;
    DcMotor ShooterB;
    DcMotor ManIn;
    DcMotor ManLift;
    Servo ManBeaconL;
    Servo ManBeaconR;
    Servo AutoBeaconL;
    Servo AutoBeaconR;
    boolean stop;
    boolean gate;
    int direction = 1;
    long shootTimeSTART;
    long shootTimeEND;
    long shootTimeDURATION;
    long currentTime;
    long lastTime;
    //Since Toggle A activates halfspeed AND Start A starts robot, halfspeed starts true to it's immediately deactivated
    boolean halfspeed = true;
    final double HALFSPEED = .35;
    final double FULLSPEED = 1;
    final long DURATION = 250;
    double speed = 0;
    MotorScaling scale;
    double voltage;
    @Override
    public void init() {
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        ShooterB = hardwareMap.dcMotor.get("B");
        ShooterF = hardwareMap.dcMotor.get("F");
        ManIn = hardwareMap.dcMotor.get("ManIn");
        ManLift = hardwareMap.dcMotor.get("ManLift");
        ManBeaconL = hardwareMap.servo.get("ManBeaconL");
        ManBeaconR = hardwareMap.servo.get("ManBeaconR");
        AutoBeaconL = hardwareMap.servo.get("AutoBeaconL");
        AutoBeaconR = hardwareMap.servo.get("AutoBeaconR");
        ManBeaconL.setPosition(.85);
        ManBeaconR.setPosition(.25);
        AutoBeaconL.setPosition(0);
        AutoBeaconR.setPosition(0);
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        ShooterB.setPower(0);
        ShooterF.setPower(0);
        ManLift.setPower(0);
        ManIn.setPower(0);
        ManIn.setPower(0);
        shootTimeSTART = 0;
        shootTimeEND = 0;
        shootTimeDURATION = 500;
        currentTime = 0;
        lastTime = 0;
        stop = false;
        gate = false;
        scale = new MotorScaling();
        voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
    }

    public double getShootingPower() throws InterruptedException {
        voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        if(voltage > 13.5) {
            return (1 - ((voltage - 13.5) * .25));
        }
        else if (voltage < 13.5) {
            return (1 + ((13.5 - voltage)) * .15);
        }
        return 1;
    }

    @Override
    public void loop() {
        //CONTROLLER 1
        //Tank Drive
        if (Math.abs(gamepad1.left_stick_y) > .1) {
            FL.setPower(gamepad1.left_stick_y * direction * -1 * speed);
            BL.setPower(gamepad1.left_stick_y * direction * -1 * speed);
        }
        else {
            FL.setPower(0);
            BL.setPower(0);
        }
        if (Math.abs(gamepad1.right_stick_y) > .1) {
            FR.setPower(gamepad1.right_stick_y * direction * speed);
            BR.setPower(gamepad1.right_stick_y * direction * speed);
        }
        else {
            FR.setPower(0);
            BR.setPower(0);
        }

//        if (Math.abs(gamepad1.left_stick_y) > .1) {
//            FL.setPower(gamepad1.left_stick_y * direction * -1 * speed);
//            BL.setPower(gamepad1.left_stick_y * direction * -1 * speed);
//        } else {
//            FL.setPower(0);
//            BL.setPower(0);
//        }
//        if (Math.abs(gamepad1.right_stick_y) > .1) {
//            FR.setPower(gamepad1.right_stick_y * direction * speed);
//            BR.setPower(gamepad1.right_stick_y * direction * speed);
//        } else {
//            FR.setPower(0);
//            BR.setPower(0);
//        }

        //HalfSpeed Macro
        if (gamepad1.a) {
            currentTime = System.currentTimeMillis();
            if (currentTime > lastTime + DURATION) {
                if (halfspeed)
                    halfspeed = false;
                else
                    halfspeed = true;
            }
            speed = (halfspeed)? HALFSPEED:FULLSPEED;
            lastTime = System.currentTimeMillis();
        }
        //Reverse Macro
        if (gamepad1.y) {
            currentTime = System.currentTimeMillis();
            if (currentTime > lastTime + DURATION)
                direction *= -1;
            lastTime = System.currentTimeMillis();
        }

        //CONTROLLER 2

        //Manipulator Control
        if (Math.abs(gamepad2.right_stick_y) > .1)
            ManIn.setPower(gamepad2.right_stick_y);

        //Half Power Manipulator
        else if (Math.abs(gamepad2.left_stick_y) > .1)
            ManIn.setPower(gamepad2.left_stick_y * .25);
        else
            ManIn.setPower(0);

        if (Math.abs(gamepad2.left_trigger) > .05)
            ManLift.setPower(gamepad2.left_trigger * .25 * -1);
        else if (Math.abs(gamepad2.right_trigger) > .05)
            ManLift.setPower(gamepad2.right_trigger * .25);
        else
            ManLift.setPower(0);

        //Back Beacon Control
        if (gamepad2.a) {
            ManBeaconL.setPosition(.3);
            ManBeaconR.setPosition(.7);
        }
        else {
            ManBeaconL.setPosition(.85);
            ManBeaconR.setPosition(.25);
        }

        //Shooter Controls
        if (gamepad2.right_bumper) {
            ShooterF.setPower(1);
            ShooterB.setPower(-1);
        }
        else if (gamepad2.left_bumper) {
            ShooterF.setPower(.9);
            ShooterB.setPower(-.9);
        }
        else {
            ShooterF.setPower(0);
            ShooterB.setPower(0);
        }

        /* //BEACON PUSH TEST
        if (gamepad2.a) {
            if(Pos<1) {
                Pos += .1;
            }
        }
        else if (gamepad2.y) {
            if (Pos>0) Pos -= .1;
        }
        ManBeaconL.setPosition(Pos);
        ManBeaconR.setPosition(1-Pos);
        telemetry.addData("Position", Pos);
        telemetry.update();

           */
        //Side Beacon Control
        if (gamepad2.x) {
            AutoBeaconL.setPosition(1);
            AutoBeaconR.setPosition(1);
        }
        else {
            AutoBeaconL.setPosition(0);
            AutoBeaconR.setPosition(0);
        }
    }
}
