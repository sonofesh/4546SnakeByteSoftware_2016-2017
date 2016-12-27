package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 11/23/16.
 */
@Autonomous(name = "Cocaine", group = "Autonomous")
public class WhiteLines extends AutoOpMode
{
    public WhiteLines(){ super(); }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("test12", "init");
        telemetry.update();
        waitForStart();
        //moveToWhiteLine(300);
        sleep(5000);
        moveForwardPID(2000);
        sleep(1000);
        //moveToWhiteLine(300);
    }
}
