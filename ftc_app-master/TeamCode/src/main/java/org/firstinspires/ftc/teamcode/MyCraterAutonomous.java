package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "States Crater Autonomous", group = "autonomous")
public class MyCraterAutonomous extends MyAutonomousBase {

    @Override
    public void runOpMode() {

        initialization();

        waitForStart();

        unlatch();

        encoderDriveAdj(-3, 0.25, 0, 2);

        String goldPos = detectMinerals();

        telemetry.addData(">","gold Pos = " + goldPos);
        telemetry.update();

        /*craterSample(goldPos);
*/
        }

}