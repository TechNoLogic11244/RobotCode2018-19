package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "My Depot  Autonomous", group = "autonomous")
public class MyDepotAutonomous extends MyAutonomousBase {

    @Override
    public void runOpMode() {

        initialization();

        waitForStart();

        /*T: comment these out
        double targetTime = runtime.seconds() + 2.5;

        while (runtime.seconds() < targetTime) {
            robot.rotationMotor.setPower(0.5);
        }

        robot.rotationMotor.setPower(0.0);

        sleep(500);

        encoderDrive(-10, 0.4);
        */
        double targetTime; 
        
        double armRotationOpenTime = 2.5;
        //double armRotationCloseTime = 1.75;
        double armExtentionOutTime = 2.0;
        //double armRetriveBackTime = 1.0;
        double robotFirstTurnDegree = 10;
        double robotFirstDriveOutDist = 5;
        
        
/*        		
       
        //Option1: Landing --- could make as a function and move to base
        //1.1-Arm Rotation to open
        targetTime = runtime.seconds() + armRotationOpenTime;
        while (runtime.seconds() < targetTime) {
            robot.rotationMotor.setPower(0.5);
        }
        robot.rotationMotor.setPower(0.0);
        
        //1.2-Arm Extention out
        targetTime = runtime.seconds() + armExtentionOutTime;
        while (runtime.seconds() < targetTime) {
            robot.extensionMotor.setPower(0.75);
        }
        robot.extensionMotor.setPower(0.0);
        
        //1.3-Robot Rotate, Move out 
        //Need test here
        turnRight(robotFirstTurnDegree, 0.75);
        encoderDrive(robotFirstDriveOutDist, 0.75);
        
        //Turn to make camera face minerals
        turnRight(robotFirstTurnDegree, 0.75);
        
        //Arm close/open further and retrive back
        
*/        
        
        
        //Option2: no landing, sampling directly
        encoderDrive(-5, 0.4);    //old way is 10, now, can be -5
        
        String goldPos = detectMinerals();

        telemetry.addData(">","gold Pos = " + goldPos);

        sampleDepot(goldPos);

        encoderDriveWithTime(40, 1.0, 6);

    }

}