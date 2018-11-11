package org.firstinspires.ftc.teamcode;

public class CraterAutonomous extends AutonomousBase {

    @Override
    public void runOpMode() {

        initialization();

        waitForStart();

        unlatch();

        //drive forward to position one
        encoderDrive(30, 0.4);

        sample();

        //drive backwards to initial position
        encoderDrive(-36, 0.7);

        //turn left to face towards wall
        turnLeft(90, 0.6);

        //drive forward
        encoderDrive(48, 0.7);

        //turn to face depot
        turnLeft(90, 0.6);

        //drive to depot
        encoderDrive(48, 0.7);

        //place marker

        //drive to crater to park
        encoderDrive(-72, 0.7);

    }

}
