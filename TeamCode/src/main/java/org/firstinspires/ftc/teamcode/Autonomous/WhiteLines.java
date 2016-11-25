package org.firstinspires.ftc.teamcode.Autonomous;

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
 * Created by sopa on 11/23/16.
 */
@Autonomous(name = "Cocaine", group = "Autonomous")
public class WhiteLines extends LinearOpMode
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
    double beforeAngle = 0;
    final double whiteACV = 27;
    final double blueACV = 0; //random number need to collect
    final double redACV = 0;
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
        beforeTime = System.currentTimeMillis();
        ShooterF.setPower(power);
        ShooterB.setPower(-power);
        bringDownShooter(.2 * -1, distance);
        while(Math.abs(System.currentTimeMillis() - beforeTime) < 5000)
            idle();
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

    public double colorSensorAverageValues(ColorSensor sensor) throws InterruptedException
    {
        double average = (sensor.red() + sensor.blue() + sensor.green())/3.0;
        return average;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        ShooterB = hardwareMap.dcMotor.get("B");
        ShooterF = hardwareMap.dcMotor.get("F");
        ManLift = hardwareMap.dcMotor.get("ManLift");
        ManIn = hardwareMap.dcMotor.get("ManIn");
        Beacon = hardwareMap.servo.get("Beacon");
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ManLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ManLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("gyro", "initalizing");
        telemetry.update();
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
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
        telemetry.addData("test17", "initalized");
        telemetry.update();
        waitForStart();
        telemetry.addData("WhiteLine", colorSensorAverageValues(colorSensorWL));
        telemetry.update();
        sleep(3000);
        telemetry.addData("Beacon", colorSensorAverageValues(colorSensorBeacon));
        telemetry.update();
        sleep(3000);

        beforeALV = getAvg();
        //barely move forward until white line is sensed
        while(Math.abs(colorSensorAverageValues(colorSensorWL) - whiteACV) > 10 && getAvg() <  beforeALV + 300)
        {
            moveForward(.15);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("encodersA", getAvg());
        telemetry.addData("colorAverage", colorSensorAverageValues(colorSensorWL));
        telemetry.update();
        sleep(250);
        
        /* Comment in as soon as the last part works
        //move forward and push the correct beacon
        if(Math.abs(blueACV - colorSensorAverageValues(colorSensorBeacon)) < Math.abs(redACV - colorSensorAverageValues(colorSensorBeacon)))
        {
            Beacon.setPosition(.8);
            beforeALV = getAvg();
            while(getAvg() <  beforeALV + 300)
            {
                moveForward(.15);
                idle();
            }
            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
            telemetry.addData("encodersA", getAvg());
            Beacon.setPosition(.6);
        }
        else
        {
            Beacon.setPosition(.8);
            beforeALV = getAvg();
            while(getAvg() <  beforeALV + 600)
            {
                moveForward(.15);
                idle();
            }
            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
            telemetry.addData("encodersA", getAvg());
            Beacon.setPosition(.6);
        }
        sleep(250);

        //forward until the next white line is sensed
        while(Math.abs(colorSensorAverageValues(colorSensorWL) - whiteACV) > 10 && getAvg() <  beforeALV + 2000)
        {
            moveForward(.15);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("encodersA", getAvg());
        telemetry.addData("colorAverage", colorSensorAverageValues(colorSensorWL));
        telemetry.update();
        sleep(250);
        //move forward and push the correct beacon
        if(Math.abs(blueACV - colorSensorAverageValues(colorSensorBeacon)) < Math.abs(redACV - colorSensorAverageValues(colorSensorBeacon)))
        {
            Beacon.setPosition(.8);
            beforeALV = getAvg();
            while(getAvg() <  beforeALV + 300)
            {
                moveForward(.15);
                idle();
            }
            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
            telemetry.addData("encodersA", getAvg());
            Beacon.setPosition(.6);
        }
        else
        {
            Beacon.setPosition(.8);
            beforeALV = getAvg();
            while(getAvg() <  beforeALV + 600)
            {
                moveForward(.15);
                idle();
            }
            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
            telemetry.addData("encodersA", getAvg());
            Beacon.setPosition(.6);
        }
        */
    }
}
