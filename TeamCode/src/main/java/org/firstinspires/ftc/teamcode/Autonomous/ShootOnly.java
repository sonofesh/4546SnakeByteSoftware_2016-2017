package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.SystemClock;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by sopa on 11/9/16.
 */
@Autonomous(name = "ShootThenKnockBigBall", group = "Autonomous")
public class ShootOnly extends LinearOpMode
{
    DcMotor FR;
    DcMotor BR;
    DcMotor FL;
    DcMotor BL;
    DcMotor ShooterB;
    DcMotor ShooterF;
    DcMotor ManLift;
    DcMotor ManIn;
    Servo Beacon;
    //average encoder value
    int beforeALV = 0;
    int beforeMLV = 0;
    double beforeAngle = 2;
    final double blackAVC = 2;
    final double whiteACV = 27;
    int FRV = 0;
    int FLV = 0;
    int avg = 0;
    long beforeTime = 0;
    long currentTime = 0;
    public BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    ColorSensor colorSensorWL;
    ColorSensor colorSensorBeacon;
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
        FL.setPower(power);
        BL.setPower(power);
    }

    public void turnLeft(double power) throws InterruptedException
    {
        turnRight(-power);
    }

    //gyro methods
    public float getGryoYaw() throws InterruptedException
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
        int beforeManLift = ManLift.getCurrentPosition();
        ShooterF.setPower(power);
        ShooterB.setPower(-power);
        bringDownShooter((.21 * -1), (distance + (Math.abs(beforeManLift - 1100))));
        sleep(1000);
        beforeTime = System.currentTimeMillis();
        while(Math.abs(System.currentTimeMillis() - beforeTime) < 3000)
            idle();
        ManIn.setPower(-.15);
        beforeTime = System.currentTimeMillis();
        while(Math.abs(System.currentTimeMillis() - beforeTime) < 500)
            idle();
        beforeTime = System.currentTimeMillis();
        while(Math.abs(System.currentTimeMillis() - beforeTime) < 2000)
            idle();
        ManIn.setPower(0);
        ShooterF.setPower(0);
        ShooterB.setPower(0);
        ManIn.setPower(0);
    }

    public int getAvg() throws InterruptedException
    {
        FRV = Math.abs(FR.getCurrentPosition());
        FLV = Math.abs(FL.getCurrentPosition());
        avg = Math.abs((FRV + FLV)/2);
        return avg;
    }

    public double colorSensorAverageValues(ColorSensor sensor) throws InterruptedException
    {
        double average = (sensor.red() + sensor.blue() + sensor.green())/3.0;
        return average;
    }
    public double colorSensorRed() throws InterruptedException
    {
        return colorSensorBeacon.red();
    }
    public double colorSensorBlue() throws InterruptedException
    {
        return colorSensorBeacon.blue();
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
        Beacon = hardwareMap.servo.get("Beacon");
        ManIn = hardwareMap.dcMotor.get("ManIn");
        Beacon.setPosition(1);
        //Delayer.setPosition(.9);
        //Stopper.setPosition(0);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ManLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ManLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("gyro", "initalizing");
        telemetry.update();
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        imu.initialize(parameters);
        telemetry.addData("gyro", "initalized");
        telemetry.update();
        colorSensorWL = hardwareMap.colorSensor.get("cSWL");
        colorSensorWL.setI2cAddress(I2cAddr.create8bit(0x2a));
        telemetry.addData("colorSensorL", "initalized");
        telemetry.update();
        colorSensorBeacon = hardwareMap.colorSensor.get("cSB");
        colorSensorBeacon.setI2cAddress(I2cAddr.create8bit(0x3c));
        telemetry.addData("colorSensorB", "initalized");
        telemetry.update();
        telemetry.addData("test1", "initalized");
        telemetry.update();
        waitForStart();
        //beforeALV = getAvg();
        //wait(15000);
        telemetry.addData("encodersA", getAvg());
        telemetry.update();
        beforeALV = getAvg();
        //original moveForward
        while(Math.abs(getAvg() - beforeALV) < 1100)
        {
            moveForward(.15);
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
        sleep(1000);
        //bring down shooter
        bringDownShooter(.1, 900);
        sleep(1500);
        //shoot
        shoot(1, 400);
        sleep(250);
        beforeALV = getAvg();
        while(Math.abs(getAvg() - beforeALV) < 3000)
        {
            moveForward(.4);
            //telemetry.addData("distance", Math.abs(System.currentTimeMillis() - beforeTime));
            //telemetry.update();
            idle();
        }
        telemetry.addData("encodersAvg", getAvg());
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        sleep(1000);







//        //turn before shooting
//        beforeAngle = getGryoYaw();
//        while(Math.abs(getGryoYaw() - beforeAngle) < 15)
//        {
//            turnRight(.25);
//            //telemetry.addData("distance", Math.abs(System.currentTimeMillis() - beforeTime));
//            //telemetry.update();
//            idle();
//        }
//        FR.setPower(0);
//        BR.setPower(0);
//        FL.setPower(0);
//        BL.setPower(0);
//        telemetry.addData("yaw angle", getGryoYaw());
//        telemetry.update();
//        sleep(1000);

//        beforeALV = getAvg();
//        while(Math.abs(getAvg() - beforeALV) < 6000)
//        {
//            moveBackward(.2);
//            //telemetry.addData("distance", Math.abs(System.currentTimeMillis() - beforeTime));
//            //telemetry.update();
//            idle();
//        }
//        telemetry.addData("encodersAvg", getAvg());
//        FR.setPower(0);
//        BR.setPower(0);
//        FL.setPower(0);
//        BL.setPower(0);
//        sleep(1000);
        //turn after shooting and before moving
//        beforeAngle = getGryoYaw();
//        while(Math.abs(getGryoYaw() - beforeAngle) < 30)
//        {
//            turnLeft(.3);
//            //telemetry.addData("distance", Math.abs(System.currentTimeMillis() - beforeTime));
//            //telemetry.update();
//            idle();
//        }
//        FR.setPower(0);
//        BR.setPower(0);
//        FL.setPower(0);
//        BL.setPower(0);
//        telemetry.addData("yaw angle", getGryoYaw());
//        telemetry.update();
        //move across the field
        //make a 165˚ turn so the robot is facing the right direction
//        Delayer.setPosition(.5);
//        beforeALV = getAvg();
//        beforeAngle = getGryoYaw();
//        while(Math.abs(getGryoYaw() - beforeAngle) < 150)
//        {
//            turnLeft(.3);
//            //telemetry.addData("distance", Math.abs(System.currentTimeMillis() - beforeTime));
//            //telemetry.update();
//            idle();
//        }
//        FR.setPower(0);
//        BR.setPower(0);
//        FL.setPower(0);
//        BL.setPower(0);
//        telemetry.addData("yaw angle", getGryoYaw());
//        telemetry.update();
//        //barely move forward until white line is sensed
//        while(Math.abs(colorSensorAverageValues(colorSensorWL) - whiteACV) > 10 && getAvg() <  beforeALV + 400)
//        {
//            moveForward(.15);
//            idle();
//        }
//        FR.setPower(0);
//        BR.setPower(0);
//        FL.setPower(0);
//        BL.setPower(0);
//        telemetry.addData("encodersA", getAvg());
//        telemetry.addData("colorAverage", colorSensorAverageValues(colorSensorWL));
//        telemetry.update();
//        sleep(250);

//        if(colorSensorRed() > colorSensorBlue())
//        {
//            Beacon.setPosition(.5);
//            beforeALV = getAvg();
//            while(getAvg() <  beforeALV + 100)
//            {
//                moveBackward(.15);
//                idle();
//            }
//            FR.setPower(0);
//            BR.setPower(0);
//            FL.setPower(0);
//            BL.setPower(0);
//            telemetry.addData("encodersA", getAvg());
//            Beacon.setPosition(.8);
//            beforeALV = getAvg();
//            while(getAvg() <  beforeALV + 100)
//            {
//                moveForward(.15);
//                idle();
//            }
//            FR.setPower(0);
//            BR.setPower(0);
//            FL.setPower(0);
//            BL.setPower(0);
//        }
//        else
//        {
//            Beacon.setPosition(.5);
//            beforeALV = getAvg();
//            while(getAvg() <  beforeALV + 200)
//            {
//                moveForward(.15);
//                idle();
//            }
//            FR.setPower(0);
//            BR.setPower(0);
//            FL.setPower(0);
//            BL.setPower(0);
//            telemetry.addData("encodersA", getAvg());
//            beforeALV = getAvg();
//            /*while(getAvg() <  beforeALV + 300)
//            {
//                moveForward(.15);
//                idle();
//            }
//            FR.setPower(0);
//            BR.setPower(0);
//            FL.setPower(0);
//            BL.setPower(0);
//            telemetry.addData("encodersA", getAvg());
//            Beacon.setPosition(.6);
//            */
//        }
//        sleep(250);
//
//        //forward until the next white line is sensed
//        while(Math.abs(colorSensorAverageValues(colorSensorWL) - whiteACV) > 10 && getAvg() <  beforeALV + 1500)
//        {
//            moveForward(.15);
//            idle();
//        }
//        FR.setPower(0);
//        BR.setPower(0);
//        FL.setPower(0);
//        BL.setPower(0);
//        telemetry.addData("encodersA", getAvg());
//        telemetry.addData("colorAverage", colorSensorAverageValues(colorSensorWL));
//        telemetry.update();
//        sleep(250);
//        //move forward and push the correct beacon
//        if(colorSensorRed() > colorSensorBlue())
//        {
//            Beacon.setPosition(.5);
//            beforeALV = getAvg();
//            while(getAvg() <  beforeALV + 300)
//            {
//                moveBackward(.15);
//                idle();
//            }
//            FR.setPower(0);
//            BR.setPower(0);
//            FL.setPower(0);
//            BL.setPower(0);
//            telemetry.addData("encodersA", getAvg());
//            Beacon.setPosition(.8);
//        }
//        else {
//            Beacon.setPosition(.5);
//            beforeALV = getAvg();
//            while (getAvg() < beforeALV + 200) {
//                moveForward(.15);
//                idle();
//            }
//            FR.setPower(0);
//            BR.setPower(0);
//            FL.setPower(0);
//            BL.setPower(0);
//            telemetry.addData("encodersA", getAvg());
//            beforeALV = getAvg();
//        }
    }
}
