package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by 4546 on 1/4/2017.
 * Does what the name says
 * Will consistently score 40 points and is a good option
 * if we're with a team that has an amazing auto(ex: Faltech)
 * Same as other ShootOnly but written by extending AutOpMode
 */

@Autonomous(name = "40PointAuto", group = "Autonomous")
public class ShootOnlyNew extends AutoOpMode {
    public ShootOnlyNew() {
        super();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        double power = .9;
        waitForStart();
        moveForward(.15, 1100);
        sleep(1000);
        bringDownShooter(.1, 900);
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
        shoot(power, 400);
        sleep(500);
        moveForward(.4, 3000);
    }
}
