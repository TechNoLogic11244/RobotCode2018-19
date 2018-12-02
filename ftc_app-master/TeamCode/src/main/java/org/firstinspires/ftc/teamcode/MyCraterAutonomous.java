package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "My Crater Autonomous", group = "autonomous")
public class MyCraterAutonomous extends MyAutonomousBase {

    @Override
    public void runOpMode() {

        initialization();

        waitForStart();

        double targetTime = runtime.seconds() + 4;

        while (runtime.seconds() < targetTime) {
            robot.extensionMotor.setPower(-0.5);
        }

        robot.extensionMotor.setPower(0.0);

        sleep(500);

        //encoderDrive(-5, 0.4);

        String goldPos = detectMinerals();

        telemetry.addData(">","gold Pos = " + goldPos);

        sampleCrater(goldPos);

        telemetry.addData(">","gold Pos = " + goldPos);
/*
        double targetTime = runtime.seconds() + 2.5;

        while (runtime.seconds() < targetTime) {
            robot.rotationMotor.setPower(0.5);
        }

        robot.rotationMotor.setPower(0.0);

        sleep(500);

        encoderDrive(-10, 0.4);

        String goldPos = detectMinerals();

        telemetry.addData(">","gold Pos = " + goldPos);

        sampleCrater(goldPos);
*/
    }

}