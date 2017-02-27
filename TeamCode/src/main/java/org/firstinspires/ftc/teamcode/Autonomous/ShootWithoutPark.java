package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by 4546 on 1/4/2017.
 * Does what the name says
 * Will consistently score 30 points and is a good option
 * if we're with a team that has an amazing auto(ex: Faltech)
 * Same as other ShootOnly but written by extending AutOpMode
 */

@Autonomous(name = "ShootNotKnock", group = "Autonomous")
public class ShootWithoutPark extends AutoOpMode {
    public ShootWithoutPark() {
        super();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("final", "init");
        telemetry.update();
        double power = .85;
        waitForStart();
        moveForwardWithEncoders(.175, 1100);
        sleep(1000);
        bringDownShooter(.275, 1150);
        sleep(1000);
        //Voltage Scaling
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        if(voltage <= 13.5)
            power = 1;
        else if(voltage <= 13.75 && voltage > 13.5)
            power = .95;
        else if(voltage > 13.75 && voltage <= 14)
            power = .9;
        else if(voltage > 14)
            power = .85;
        shoot(power, 360);
        sleep(500);
    }
}

