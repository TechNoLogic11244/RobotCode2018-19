package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        robot.rightRear.setDirection(DcMotor.Direction.REVERSE);
        robot.leftRear.setDirection(DcMotor.Direction.FORWARD);

        robot.extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void sampleDepot(String goldPos) {

        if (goldPos == "right") {
            turnRight(13, 0.75);
            encoderDrive(40, 0.75);
            /*double targetTime = runtime.seconds() + 1.75;

            while (runtime.seconds() < targetTime) {
                robot.rotationMotor.setPower(-0.5);
            }
            robot.rotationMotor.setPower(0.0);
            */
            sleep(250);
            turnLeft(18, 0.75);
            sleep(250);
            encoderDrive(24, 0.75);
            sleep(250);
            placeMarker();
            encoderDrive(-35, 0.75);
            turnRight(95, 0.75);
        } else if (goldPos == "left") {
            turnLeft(13, 0.75);
            encoderDrive(40, 0.75);
            /*
            double targetTime = runtime.seconds() + 1.75;

            while (runtime.seconds() < targetTime) {
                robot.rotationMotor.setPower(-0.5);
            }
            robot.rotationMotor.setPower(0.0);
            */
            sleep(250);
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
            encoderDrive(55, 0.75);
            /*
            double targetTime = runtime.seconds() + 1.75;

            while (runtime.seconds() < targetTime) {
                robot.rotationMotor.setPower(-0.5);
            }
            robot.rotationMotor.setPower(0.0);
            */
            sleep(250);
            placeMarker();
            encoderDrive(-30, 0.75);
            turnRight(60, 0.75);
            encoderDrive(33, 0.75);
            turnRight(7, 0.5);
        }
    }

    //adjust later to add team marker logic
    public void sampleCrater(String goldPos){

        if (goldPos == "right"){
            turnRight(20, 0.75);
            encoderDriveWithTime(36, 0.75,6);
        } else if (goldPos == "left"){
            turnLeft(20, 0.75);
            encoderDriveWithTime(36, 0.75, 6);
        } else {
            encoderDriveWithTime(33, 0.75, 6);
            turnLeft(20, 0.5);
            encoderDriveWithTime(2, 0.5, 2);
        }

    }

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
                    }
                }
				else if (updatedRecognitions.size() == 2)
				{
					int goldMineralX = -1;
    				int silverMineral1X = -1;
    				int silverMineral2X = -1;

    				// This just records values, and is unchanged
				    for (Recognition recognition : updatedRecognitions) 
				    {
				        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) 
				        {
				            goldMineralX = (int) recognition.getLeft();
				        } 
				        else if (silverMineral1X == -1) 
				        {
				            silverMineral1X = (int) recognition.getLeft();
				        }    
				        else 
				        {
				            silverMineral2X = (int) recognition.getLeft();
				        }
				    }

					//Our robot unlatches by driving slightly to the left (with mecanum wheels) so it can 
					//see the left two minerals only, but the phone camera could be mounted further to 
					//the left side instead. If mounting to right, need change 'right' to 'left'
				    // If there is no gold (-1) and there two silvers (not -1) the gold
				    // is not visible, and must be on the right
				    if (goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1)
				    {
				        goldPos = "right";
				    }
				    // If you can see one gold and one silver ...
				    else if (goldMineralX != -1 && silverMineral1X != -1)
				    {
				        // ... if the gold is to the right of the silver, the gold is in the center ...
				        if (goldMineralX > silverMineral1X) 
				        {
				            goldPos = "center";
				        } 
				        // ... otherwise it is on the left
				        else 
				        {
				            goldPos = "left";
				        }
				    }
				}
				//The only 1 detected could be moved to other function to make sure more accurate result
				else if (updatedRecognitions.size() == 1)
				{
					int goldMineralX = -1;
    				int silverMineral1X = -1;
    				int silverMineral2X = -1;

    				// This just records values, and is unchanged
				    for (Recognition recognition : updatedRecognitions) 
				    {
				        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) 
				        {
				            goldMineralX = (int) recognition.getLeft();
				        } 
				        else if (silverMineral1X == -1) 
				        {
				            silverMineral1X = (int) recognition.getLeft();
				        }    
				        else 
				        {
				            silverMineral2X = (int) recognition.getLeft();
				        }
				    }

				    // If there is a gold (not -1) 
				    if (goldMineralX != -1 )
				    {
				        goldPos = "center";
				    }
				    // If you can see one silver, just guess to get 50%
				    else 
					{
						goldPos = "left";
				    }
				}
				
				if (goldPos != "unknown")
				{
					break;
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

    public void placeMarker(){
        telemetry.addData(">", "Placing Team Marker...");
        telemetry.update();
        sleep(1000);
    }

    // power should always be POSITIVE
    // distance should be in INCHES
    // positive distance indicates driving forward
    // negative distance indicates driving backwards
    public void encoderDriveWithTime(double distance, double maxPwr, double time) {

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

        double initTime = runtime.seconds();
        double targetTime = initTime + time;

        while (Math.abs(currentRobotPos) < Math.abs(targetPos) && runtime.seconds() < targetTime){

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

}
