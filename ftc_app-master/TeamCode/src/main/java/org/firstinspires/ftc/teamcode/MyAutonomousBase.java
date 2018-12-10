package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

public class MyAutonomousBase extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    TFlowVuforiaBase tfv = new TFlowVuforiaBase();
    ElapsedTime runtime = new ElapsedTime();

    Orientation angles;

    final double GYRO_CORRECTION_FACTOR = 0.015;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void initialization() {

        robot.leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        robot.rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        robot.extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
        robot.rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");

        robot.leftClaw = hardwareMap.servo.get("leftClaw");
        robot.rightClaw = hardwareMap.servo.get("rightClaw");
        robot.markerServo = hardwareMap.servo.get("markerServo");

        robot.rightRear.setDirection(DcMotor.Direction.REVERSE);
        robot.leftRear.setDirection(DcMotor.Direction.FORWARD);

        robot.extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;

        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters);

        while (!robot.imu.isGyroCalibrated()){
            telemetry.addData("gyro not calibrated", "do not touch!");
            telemetry.update();
        }

        telemetry.addData(">","press start to start");
        telemetry.update();

        robot.leftClaw.setPosition(0.0);
        robot.rightClaw.setPosition(1.0);
        robot.markerServo.setPosition(robot.MARKER_START);

        tfv.initTFlowAndVuforia();

        runtime.reset();
    }

    // power should always be POSITIVE
    // distance should be in INCHES
    // positive distance indicates driving forward
    // negative distance indicates driving backwards
    public void encoderDrive(double distance, double maxPwr) {

        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double initPosR = robot.rightRear.getCurrentPosition();
        double initPosL = robot.leftRear.getCurrentPosition();

        double targetPos    = distance * robot.COUNTS_PER_INCH;
        double distanceSign = Math.signum(distance);

        double currentRobotPos = 0.;

        double power = maxPwr;

        // slopes for proportional speed increase/decrease
        double decSlope = (maxPwr - robot.MINIMUM_DRIVE_PWR) / (robot.DECELERATION_THRESHOLD);

        while (Math.abs(currentRobotPos) < Math.abs(targetPos)){

            double curPosR = robot.rightRear.getCurrentPosition() - initPosR;
            double curPosL = robot.leftRear.getCurrentPosition() - initPosL;

            currentRobotPos = (curPosR + curPosL) / 2;

            // calculating points on trapezoidal profile graph
            power = maxPwr - decSlope * (Math.abs(currentRobotPos) / robot.COUNTS_PER_INCH);

            if (power < robot.MINIMUM_DRIVE_PWR)
                power = robot.MINIMUM_DRIVE_PWR;

            robot.rightRear.setPower(distanceSign * power);
            robot.leftRear.setPower(distanceSign * power);

            telemetry.addData(">", "target position = " + targetPos);
            telemetry.addData(">", "current robot pos = " + currentRobotPos);
            telemetry.addData(">", "right motor encoder pos = " + curPosR);
            telemetry.addData(">", "left motor encoder pos = " + curPosL);
            telemetry.update();

        }

        robot.rightRear.setPower(0.);
        robot.leftRear.setPower(0.);

    }
	
	// power should always be POSITIVE
    // distance should be in INCHES
    // positive distance indicates driving forward
    // negative distance indicates driving backwards
/*
    public void encoderArmExtenstion(double distance, double maxPwr) {
        robot.extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double initPos = robot.extensionMotor.getCurrentPosition();

        double targetPos    = distance * robot.COUNTS_PER_INCH;
        double distanceSign = Math.signum(distance);
        double currentArmPos = 0.;
        double power = maxPwr;
        // slopes for proportional speed increase/decrease
        double decSlope = (maxPwr - robot.MINIMUM_DRIVE_PWR) / (robot.DECELERATION_THRESHOLD);
        while (Math.abs(currentRobotPos) < Math.abs(targetPos)){
            double curPos = robot.extensionMotor.getCurrentPosition() - initPos;

            currentArmPos = curPos;
            // calculating points on trapezoidal profile graph
            power = maxPwr - decSlope * (Math.abs(currentArmPos) / robot.COUNTS_PER_INCH);
            if (power < robot.MINIMUM_DRIVE_PWR)
                power = robot.MINIMUM_DRIVE_PWR;
            robot.extensionMotor.setPower(distanceSign * power);

            telemetry.addData(">", "target Arm position = " + targetPos);
            telemetry.addData(">", "current Arm pos = " + currentRobotPos);
            telemetry.addData(">", "Arm motor encoder pos = " + curPos);
            telemetry.update();
        }
        robot.extensionMotor.setPower(0.);

    }
*/

    public void turnRight(double angle, double power){

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double initAngle      = angles.firstAngle;
        double referenceAngle = 0.0;

        double slope = (power - robot.MINIMUM_TURN_PWR) / Math.abs(angle);

        double calculatedPwr = power;

        while (referenceAngle < Math.abs(angle) && opModeIsActive()){

            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double curAngle = angles.firstAngle;

            if (curAngle * initAngle >= 0)
                referenceAngle = Math.abs(curAngle - initAngle);
            else {
                if (curAngle > 0)
                    referenceAngle = (180 - curAngle) + (initAngle + 180);
                else
                    referenceAngle = initAngle - curAngle;
            }

            calculatedPwr = power - referenceAngle * slope;

            if (calculatedPwr <= robot.MINIMUM_TURN_PWR)
                calculatedPwr = robot.MINIMUM_TURN_PWR;

            robot.rightRear.setPower(-calculatedPwr);
            robot.leftRear.setPower(calculatedPwr);

            telemetry.addData(">", "current heading = " + angles.firstAngle);
            telemetry.addData(">", "target heading = " + Math.abs(angle));
            telemetry.addData(">", "reference angle = " + referenceAngle);
            telemetry.update();

        }

        robot.rightRear.setPower(0.);
        robot.leftRear.setPower(0.);

    }

    // All parameters should be POSITIVE
    // Angle is the amount you want to turn (ex: 30 degrees)
    // Power how fast you want to turn
    public void turnLeft(double angle, double power){
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double initAngle      = angles.firstAngle;
        double referenceAngle = 0.0;

        double slope = (power - robot.MINIMUM_TURN_PWR) / Math.abs(angle);

        double calculatedPwr = power;

        while (referenceAngle < Math.abs(angle) && opModeIsActive()){

            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double curAngle = angles.firstAngle;

            if (curAngle * initAngle > 0)
                referenceAngle = Math.abs(curAngle - initAngle);
            else {
                if (curAngle >= 0)
                    referenceAngle = curAngle - initAngle;
                else
                    referenceAngle = (180 - initAngle) + (curAngle + 180);
            }

            calculatedPwr = power - referenceAngle * slope;

            if (calculatedPwr <= robot.MINIMUM_TURN_PWR)
                calculatedPwr = robot.MINIMUM_TURN_PWR;

            robot.rightRear.setPower(calculatedPwr);
            robot.leftRear.setPower(-calculatedPwr);

            telemetry.addData(">", "current heading = " + angles.firstAngle);
            telemetry.addData(">", "target heading = " + Math.abs(angle));
            telemetry.addData(">", "reference angle = " + referenceAngle);
            telemetry.update();

        }

        robot.rightRear.setPower(0.);
        robot.leftRear.setPower(0.);

    }


    /*public void sampleDepot(String goldPos) {
        if (goldPos == "right") {
            turnRight(13, 0.75);
            encoderDrive(40, 0.75);
            /*double targetTime = runtime.seconds() + 1.75;
            while (runtime.seconds() < targetTime) {
                robot.rotationMotor.setPower(-0.5);
            }
            robot.rotationMotor.setPower(0.0);
            */
            /*sleep(250);
            turnLeft(18, 0.75);
            sleep(250);
            encoderDrive(24, 0.75);
            sleep(250);
            placeMarker();
            encoderDrive(-35, 0.75);
            turnRight(95, 0.75);
        } else if (goldPos == "left") {
            turnLeft(13, 0.75);
            encoderDrive(40, 0.75);*/
            /*
            double targetTime = runtime.seconds() + 1.75;
            while (runtime.seconds() < targetTime) {
                robot.rotationMotor.setPower(-0.5);
            }
            robot.rotationMotor.setPower(0.0);
            */
            /*sleep(250);
            turnRight(14, 0.5);
            sleep(250);
            encoderDrive(24, 0.75);
            sleep(250);
            placeMarker();
            encoderDrive(-38, 0.75);
            sleep(250);
            turnRight(56, 0.75);
            encoderDrive(35, 0.75);
            turnRight(5, 0.5);
        } else {
            encoderDrive(55, 0.75);*/
            /*
            double targetTime = runtime.seconds() + 1.75;
            while (runtime.seconds() < targetTime) {
                robot.rotationMotor.setPower(-0.5);
            }
            robot.rotationMotor.setPower(0.0);
            */
            /*sleep(250);
            placeMarker();
            encoderDrive(-30, 0.75);
            turnRight(60, 0.75);
            encoderDrive(33, 0.75);
            turnRight(7, 0.5);
        }
    }*/


    public void depotSample(String goldPos) {

        if (goldPos == "right") {
            //1. turn right and forward to knock
            turnRight(25.0, 0.5);
            encoderDriveAdj(-35.0 + 2.0, 0.5, -25.0, 6.0);
            sleep(250);

            //2. backward
            encoderDriveAdj(18.0 - 2.0, 0.5, -25.0, 4.0);
            //turnLeft(18, 0.75);
            //sleep(250);

            //3. turn left
            turnLeft(108.0, 0.5);
            sleep(250);
            //encoderDrive(24, 0.75);
            //sleep(250);

            //4. forward
            encoderDriveAdj(-53.5, 0.6, 90, 7.0);
            sleep(250);

            //5. turn left facing depot
            turnRight(110, 0.6);
            sleep(250);

            //6. move forward to depot
            encoderDriveAdj(-57.0, 0.6, -43, 8.0);
            sleep(250);

            //7. claim
            placeMarker();

            //8. backward
            encoderDriveAdj(66.5, 0.7, -45, 4);
            //turnRight(95, 0.75);

            //9. parking (extent and rotate arm)
            parking();

        } else if (goldPos == "left") {
            //1. turn left and move forward to knock
            turnLeft(27.0, 0.5);
            encoderDriveAdj(-35.0 + 2.0 - 6, 0.55, 27.0, 6.0);
            sleep(250);

            //2. backward
            encoderDriveAdj(18.0 - 2.0 + 6, 0.5, 27.0, 4.0);
            //turnLeft(18, 0.75);
            sleep(250);

            //3. turn left
            turnLeft(57.0, 0.5);
            sleep(250);
            //encoderDrive(24, 0.75);
            //sleep(250);

            //4. forward
            encoderDriveAdj(-37.5, 0.6, 84, 7.0);
            sleep(250);

            //5. turn left facing depot
            turnRight(120.0, 0.6);
            sleep(250);

            //6. move forward to depot
            encoderDriveAdj(-57.0, 0.6, -45, 8.0);
            sleep(250);

            //7. claim
            placeMarker();

            //8. backward
            encoderDriveAdj(66.5, 0.7, -45, 4);
            //turnRight(95, 0.75);

            //9. parking (extent and rotate arm)
            rotateArm(4000, 0.8);
        	
        } else {
            //1. forward
            encoderDriveAdj(-52, 0.45, 0, 8);
            sleep(250);

            //2. Claming
            placeMarker();

            //3. backward
            encoderDriveAdj(43.5, 0.45, 0, 8);
            sleep(500);

            //4. turn left (~90 degree)
            turnLeft(84, 0.5);
            //encoderDrive(33, 0.75);
            //turnRight(7, 0.5);

            //5. move forward
            encoderDriveAdj(-45.0, 0.6, 84, 8);

            //6. turn right
            turnRight(125.0, 0.6);

            //7. move forward
            encoderDriveAdj(10, 0.4, -40, 2.5);

            //8. parking (extent and rotate arm)
            parking();

        }
    }

    //adjust later to add team marker logic
    public void craterSample(String goldPos){
        //From step 5 could be same if can reach the same point
        if (goldPos == "right") {
            //1. turn right and forward to knock
            turnRight(25.0, 0.5);
            encoderDriveAdj(-35.0 + 2.0 + 4.0, 0.5, -25.0, 6.0);
            sleep(250);

            //2. backward
            encoderDriveAdj(18.0 - 2.0 - 2.0, 0.5, -25.0, 4.0);
            //turnLeft(18, 0.75);
            //sleep(250);

            //3. turn left
            turnLeft(108.0, 0.5);
            sleep(250);
            //encoderDrive(24, 0.75);
            //sleep(250);

            //4. forward
            encoderDriveAdj(-54.5, 0.6, 90, 7.0);
            sleep(250);

            //5. turn left facing depot
            turnLeft(30.0, 0.6);
            sleep(250);

            //6. move forward to depot
            encoderDriveAdj(-49.5, 0.6, 138.25, 8.0);
            sleep(250);

            //7. claim
            placeMarker();

            //8. backward
            encoderDriveAdj(66.5, 0.7, 138.25, 4);
            //turnRight(95, 0.75);

            //9. parking (extent and rotate arm)
            parking();

        } else if (goldPos == "left") {
            //1. turn left and move forward to knock
            turnLeft(27.0, 0.5);
            encoderDriveAdj(-35.0 + 2.0 + 4.0 , 0.55, 27.0, 6.0);
            sleep(250);

            //2. backward
            encoderDriveAdj(18.0 - 2.0  - 2.0, 0.5, 27.0, 4.0);
            //turnLeft(18, 0.75);
            sleep(250);

            //3. turn left
            turnLeft(57.0, 0.5);
            sleep(250);
            //encoderDrive(24, 0.75);
            //sleep(250);

            //4. forward
            encoderDriveAdj(-34.5, 0.6, 84, 7.0);
            sleep(250);

            //5. turn left facing depot
            turnLeft(30.0, 0.6);
            sleep(250);

            //6. move forward to depot
            encoderDriveAdj(-57.0, 0.6, 137, 8.0);
            sleep(250);

            //7. claim
            placeMarker();

            //8. backward
            encoderDriveAdj(66.5, 0.7, 138.25, 4);
            //turnRight(95, 0.75);

            //9. parking (extent and rotate arm)
            parking();

        } else {
            //1. forward
            encoderDriveAdj(-21, 0.5, 0, 8);
            sleep(250);

            //3. backward
            encoderDriveAdj(10, 0.5, 0, 8);
            sleep(250);

            //4. turn left (~90 degree)
            turnLeft(84, 0.75);
            //encoderDrive(33, 0.75);
            //turnRight(7, 0.5);

            //5. move forward
            encoderDriveAdj(-41.75, 0.75, 84, 8);

            //6. turn left
            turnLeft(30, 0.75);

            //7. move forward
            encoderDriveAdj(-57, 0.75, 135, 8);

            placeMarker();

            encoderDriveAdj(88, 0.75, 138.25, 4);

            //8. parking (extent and rotate arm)
            parking();

        }

    }

    //adjust later to add team marker logic
    /*public void sampleCrater(String goldPos){
        if (goldPos == "right"){
            turnRight(20, 0.75);
            encoderDriveAdj(-36, 0.75,6);
        } else if (goldPos == "left"){
            turnLeft(20, 0.75);
            encoderDriveAdj(-36, 0.75, 6);
        } else {
            encoderDriveAdj(-33, 0.75, 6);
            turnLeft(20, 0.5);
            encoderDriveAdj(-2, 0.5, 2);
        }
    }*/

    public String detectMinerals(){

        String goldPos = "unknown";

        // activate tensor flow
        if (tfv.tfod != null) {
            tfv.tfod.activate();
        }

        double initTime = runtime.seconds();
        double targetTime = initTime + 3.;

        while (runtime.seconds() < targetTime){

            if (tfv.tfod != null) {

                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfv.tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    //telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {

                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;

                        // get coordinates for all minerals
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(tfv.LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }

                        // get position of gold mineral
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                goldPos = "left";
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                goldPos = "right";
                            } else {
                                goldPos = "center";
                            }
                        }
                    } else if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;

                        // This just records values, and is unchanged
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(tfv.LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }

                        //Our robot unlatches by driving slightly to the left (with mecanum wheels) so it can
                        //see the left two minerals only, but the phone camera could be mounted further to
                        //the left side instead. If mounting to right, need change 'right' to 'left'
                        // If there is no gold (-1) and there two silvers (not -1) the gold
                        // is not visible, and must be on the right
                        if (goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            goldPos = "right";
                        }
                        // If you can see one gold and one silver ...
                        else if (goldMineralX != -1 && silverMineral1X != -1) {
                            // ... if the gold is to the right of the silver, the gold is in the center ...
                            if (goldMineralX > silverMineral1X) {
                                goldPos = "center";
                            }
                            // ... otherwise it is on the left
                            else {
                                goldPos = "left";
                            }
                        }
                    }
/*
                    //The only 1 detected could be moved to other function to make sure more accurate result
                    else if (updatedRecognitions.size() == 1) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        // This just records values, and is unchanged
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(tfv.LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        // If there is a gold (not -1)
                        if (goldMineralX != -1) {
                            goldPos = "center";
                        }
                        // If you can see one silver, just guess to get 50%
                        else {
                            goldPos = "left";
                        }
                    }
*/
                    if (goldPos != "unknown") {
                        break;
                    }
                }
            }
        }

        telemetry.addData(">", "gold mineral position = " + goldPos);
        telemetry.update();

        if (tfv.tfod != null) {
            tfv.tfod.shutdown();
        }

        return goldPos;
    }

/*
    public String detectOneMineralOnly(){
        String goldPos = "unknown";
        // activate tensor flow
        if (tfv.tfod != null) {
            tfv.tfod.activate();
        }
        double initTime = runtime.seconds();
        double targetTime = initTime + 2.;
        while (runtime.seconds() < targetTime){
            if (tfv.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfv.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    //telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 1) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        // This just records values, and is unchanged
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(tfv.LABEL_GOLD_MINERAL)) {
                            	goldPos = "Found";
                            }
                        }
                    }

                    if (goldPos != "unknown") {
                        break;
                    }
                }
            }
        }
        telemetry.addData(">", "gold mineral position = " + goldPos);
        telemetry.update();
        if (tfv.tfod != null) {
            tfv.tfod.shutdown();
        }
        return goldPos;
    }
*/


    public void placeMarker(){
        telemetry.addData(">", "Placing Team Marker...");
        telemetry.update();

        robot.markerServo.setPosition(robot.MARKER_PLACE);
        sleep(500);
        robot.markerServo.setPosition(robot.MARKER_START);
        sleep(250);
    }

    // power should always be POSITIVE
    // distance should be in INCHES
    // positive distance indicates driving forward
    // negative distance indicates driving backwards
    public void encoderDriveAdj(double distance, double maxPwr, double targetHeading, double time) {

        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double initPosR = robot.rightRear.getCurrentPosition();
        double initPosL = robot.leftRear.getCurrentPosition();

        double targetPos    = distance * robot.COUNTS_PER_INCH;
        double distanceSign = Math.signum(distance);

        double currentRobotPos = 0.;

        double power = maxPwr;

        // slopes for proportional speed increase/decrease
        double decSlope = (maxPwr - robot.MINIMUM_DRIVE_PWR) / Math.abs(distance);

        double initTime = runtime.seconds();
        double targetTime = initTime + time;

        while (Math.abs(currentRobotPos) < Math.abs(targetPos) && runtime.seconds() < targetTime){

            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double curPosR = robot.rightRear.getCurrentPosition() - initPosR;
            double curPosL = robot.leftRear.getCurrentPosition() - initPosL;

            currentRobotPos = (curPosR + curPosL) / 2;

            // calculating points on trapezoidal profile graph
            power = maxPwr - decSlope * (Math.abs(currentRobotPos) / robot.COUNTS_PER_INCH);

            if (power < robot.MINIMUM_DRIVE_PWR)
                power = robot.MINIMUM_DRIVE_PWR;

            double currentHeading = angles.firstAngle;

            double gyroCorrectionPwr = GYRO_CORRECTION_FACTOR * computeGyroDriveCorrectionError(targetHeading, currentHeading);

            /*
            if ((distance <= 0 && targetHeading <= 0) || (distance <= 0 && targetHeading >= 0)) {
                robot.rightRear.setPower(distanceSign * power - (gyroCorrectionPwr * distanceSign));
                robot.leftRear.setPower(distanceSign * power + (gyroCorrectionPwr * distanceSign));
            } else {
                robot.rightRear.setPower(distanceSign * power + (gyroCorrectionPwr * distanceSign));
                robot.leftRear.setPower(distanceSign * power - (gyroCorrectionPwr * distanceSign));
            }
            */

            robot.rightRear.setPower((power - (gyroCorrectionPwr * distanceSign)) * distanceSign);
            robot.leftRear.setPower((power + (gyroCorrectionPwr * distanceSign)) * distanceSign);

            telemetry.addData(">", "target position = " + targetPos);
            telemetry.addData(">", "current robot pos = " + currentRobotPos);
            telemetry.addData(">", "right motor encoder pos = " + curPosR);
            telemetry.addData(">", "left motor encoder pos = " + curPosL);
            telemetry.addData("gyro", "current gyro heading = " + currentHeading);
            telemetry.update();

        }

        robot.rightRear.setPower(0.);
        robot.leftRear.setPower(0.);

    }

    double computeGyroDriveCorrectionError(double inputHeading, double currentHeading) {

        double error;
        if (inputHeading * currentHeading >= 0)
            error = currentHeading - inputHeading;
        else {
            if (Math.abs(inputHeading) > 90) {
                if (inputHeading < 0)
                    error = -((180 - currentHeading) + (180 + inputHeading));
                else
                    error = (180 + currentHeading) + (180 - inputHeading);
            } else
                error = currentHeading - inputHeading;
        }

        return error;
    }

    //positive counts for one way
    //negative counts for the other
    public void rotateArm(double counts, double power) {

        double initPos = robot.rotationMotor.getCurrentPosition();
        double targetPos = counts;

        double curPos = 0.0;

        double rotateSign = Math.signum(counts);

        while (Math.abs(curPos) < Math.abs(targetPos)) {

            curPos = robot.rotationMotor.getCurrentPosition() - initPos;

            robot.rotationMotor.setPower(power * rotateSign);

        }

        robot.rotationMotor.setPower(0.0);

    }

    //positive counts for one way
    //negative counts for the other
    public void extendArm(double counts, double power) {

        double initPos = robot.extensionMotor.getCurrentPosition();
        double targetPos = initPos + counts;

        double curPos = 0.0;

        double rotateSign = -Math.signum(counts);

        while (Math.abs(curPos) < Math.abs(targetPos)) {

            curPos = robot.extensionMotor.getCurrentPosition() - initPos;

            robot.extensionMotor.setPower(power * rotateSign);

        }

        robot.extensionMotor.setPower(0.0);

    }

    public void unlatch(){

        rotateArm(5834, 0.4);

        sleep(250);

        extendArm(-7689, 0.8);

    }

    public void parking() {
        //close the claw servo
        robot.leftClaw.setPosition(0.55);
        robot.rightClaw.setPosition(0.4);

        //Originally it is 4300, which is not close to the ground enough
        rotateArm(4300 + 500, 0.8);
    }


}