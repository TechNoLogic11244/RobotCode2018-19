package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Depot  Autonomous", group = "autonomous")
public class DepotAutonomous extends AutonomousBase {

    @Override
    public void runOpMode() {

        initialization();

        waitForStart();

        double targetTime = runtime.seconds() + 2.5;

        while (runtime.seconds() < targetTime) {
            robot.rotationMotor.setPower(0.5);
        }

        robot.rotationMotor.setPower(0.0);

        sleep(500);

        encoderDrive(-10, 0.4);

        String goldPos = detectMinerals();

        telemetry.addData(">","gold Pos = " + goldPos);

        sampleDepot(goldPos);

        encoderDriveWithTime(40, 1.0, 6);

    }

}
