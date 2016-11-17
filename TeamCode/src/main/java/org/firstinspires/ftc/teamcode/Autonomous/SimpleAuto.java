package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.SystemClock;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
    //average encoder value
    int beforeALV = 0;
    int beforeMLV = 0;
    double beforeAngle = 0;
    int FRV = 0;
    int FLV = 0;
    int avg = 0;
    long beforeTime = 0;
    long currentTime = 0;
    public static BNO055IMU imu;
    public static BNO055IMU.Parameters parameters;

    public void zero() throws InterruptedException
    {
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void moveForward(double power) throws InterruptedException
    {
        FR.setPower(power);
        BR.setPower(power);
        FL.setPower(-power);
        BL.setPower(-power);
    }

    public void moveBackward(double power) throws InterruptedException
    {
        moveForward(-power);
    }

    public void turnRight(double power) throws InterruptedException
    {
        FR.setPower(power);
        BR.setPower(power);
        FL.setPower(-power);
        BL.setPower(-power);
    }

    public void turnLeft(double power) throws InterruptedException
    {
        turnRight(-power);
    }

    //gyro methods
    public float getGryoYaw()
    {
        Orientation angles = imu.getAngularOrientation();
        return angles.firstAngle;
    }

    public void bringDownShooter(double power, int distance) throws InterruptedException
    {
        int beforePos = Math.abs(ManLift.getCurrentPosition());
        telemetry.addData("ManLift", ManLift.getCurrentPosition());
        telemetry.update();
        while (Math.abs(ManLift.getCurrentPosition() - beforePos) < distance)
        {
            ManLift.setPower(power);
            idle();
        }
        ManLift.setPower(0);
        telemetry.addData("ManLift", ManLift.getCurrentPosition());
        telemetry.update();
    }

    public void shoot(double power, int distance) throws InterruptedException
    {
        bringDownShooter(.3 * -1, distance);
        wait(1000);
        ShooterF.setPower(power);
        ShooterB.setPower(-power);
    }

    public void shootTime(double power) throws InterruptedException
    {
        ShooterF.setPower(power);
        ShooterB.setPower(-power);
        double startTime = System.currentTimeMillis();
        while(Math.abs(ManLift.getCurrentPosition() - startTime) < 600)
        {
            ManLift.setPower(.05);
            idle();
        }
        ManLift.setPower(0);
        wait(1000);
        ShooterF.setPower(0);
        ShooterB.setPower(0);
    }

    public int getAvg() throws InterruptedException
    {
        FRV = Math.abs(FR.getCurrentPosition());
        FLV = Math.abs(FL.getCurrentPosition());
        avg = Math.abs((FRV + FLV)/2);
        return avg;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        ShooterB = hardwareMap.dcMotor.get("B");
        ShooterF = hardwareMap.dcMotor.get("F");
        ManLift = hardwareMap.dcMotor.get("ManLift");
        ManIn = hardwareMap.dcMotor.get("ManIn");
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ManLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ManLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("gyro", "initalized");
        telemetry.update();
        waitForStart();
        //beforeALV = getAvg();
        //wait(15000);
        telemetry.addData("encodersR", FR.getCurrentPosition());
        telemetry.addData("encodersL", FL.getCurrentPosition());
        telemetry.update();
        beforeALV = getAvg();
        while(getAvg() <  beforeALV + 1000)
        {
            moveForward(.2);
            //telemetry.addData("distance", Math.abs(System.currentTimeMillis() - beforeTime));
            //telemetry.update();
            idle();
        }
        telemetry.addData("encodersR", getAvg());
        telemetry.update();
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        sleep(2000);
        //bring down shooter
        bringDownShooter(.1, 900);
        sleep(1000);
        beforeAngle = getGryoYaw();
        while(Math.abs(getGryoYaw() - beforeALV) > 20)
        {
            turnLeft(.3);
            //telemetry.addData("distance", Math.abs(System.currentTimeMillis() - beforeTime));
            //telemetry.update();
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("yaw angle", getGryoYaw());
        telemetry.update();
        sleep(1000);
        shoot(1, 400);
        ShooterB.setPower(0);
        ShooterF.setPower(0);
        //long currTime = System.currentTimeMillis();
        //if(Math.abs(currTime - startTime) > 15000)
    }
}
