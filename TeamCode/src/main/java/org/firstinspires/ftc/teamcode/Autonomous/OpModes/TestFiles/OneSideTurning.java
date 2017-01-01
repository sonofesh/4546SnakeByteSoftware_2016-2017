package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;
import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 12/31/16.
 */
public class OneSideTurning extends AutoOpMode {
    public OneSideTurning() {super(); }
    //turnRightWithPID(50, .006, .00003, 0.0);
    @Override
    public void runOpMode() throws InterruptedException {
        double beforeAngle = getGyroYaw();
        initialize();
        telemetry.addData("test1", "init");
        telemetry.update();
        turnLeftWithPIDOneSide(50, .012, .00003, 0.0);
        Thread.sleep(3000);
    }
}
