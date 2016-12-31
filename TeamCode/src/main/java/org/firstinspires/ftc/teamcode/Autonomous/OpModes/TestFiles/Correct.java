package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import android.provider.MediaStore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by 4546 SnakeByte on 12/26/16.
 */
@Autonomous(name = "Correct", group = "Autonomous")
public class Correct extends AutoOpMode {
    public Correct() {
        super();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("init", "test5");
        telemetry.update();
        double perpendicular = getGyroYaw();
        telemetry.addData("perpendicular", perpendicular);
        telemetry.update();
        waitForStart();
        //correct(perpendicular);
    }
}
