package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutonomousBase extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

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

        robot.leftRear.setDirection(DcMotor.Direction.REVERSE);

        robot.extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;

        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters);

        while (!robot.imu.isGyroCalibrated()){
            telemetry.addData("gyro not calibrated", "do not touch!");
            telemetry.update();
        }

        //set servos to initial positions
    }

        // power should always be POSITIVE
        // distance should be in INCHES
        // positive distance indicates driving forward
        // negative distance indicates driving backwards
        public void encoderDrive(double distance, double maxPwr) {

            robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double brakeDist    = (Math.abs(distance) - robot.DECELERATION_THRESHOLD) * robot.COUNTS_PER_INCH;
            double accThreshold = Math.abs(distance) * robot.ACC_CONSTANT * robot.COUNTS_PER_INCH;

            double initPosR = robot.rightRear.getCurrentPosition();
            double initPosL = robot.leftRear.getCurrentPosition();

            double targetPos    = distance * robot.COUNTS_PER_INCH;
            double distanceSign = Math.signum(distance);

            double currentRobotPos = 0.;

            double power = maxPwr;

            // slopes for proportional speed increase/decrease
            double decSlope = (maxPwr - robot.MINIMUM_DRIVE_PWR) / (robot.DECELERATION_THRESHOLD);
            double accSlope = (maxPwr - robot.MINIMUM_DRIVE_PWR) / (accThreshold);

            while (Math.abs(currentRobotPos) < Math.abs(targetPos)){

                double curPosR = robot.rightRear.getCurrentPosition() - initPosR;
                double curPosL = robot.leftRear.getCurrentPosition() - initPosL;

                currentRobotPos = (curPosR + curPosL) / 2;


                // calculating points on trapezoidal profile graph
                if (Math.abs(currentRobotPos) < accThreshold)
                    power = currentRobotPos * accSlope;
                else if (Math.abs(currentRobotPos) > brakeDist)
                    power = maxPwr - (Math.abs(currentRobotPos) - brakeDist) * decSlope;
                else
                    power = maxPwr;

                if (power < robot.MINIMUM_DRIVE_PWR)
                    power = robot.MINIMUM_DRIVE_PWR;

                robot.rightRear.setPower(distanceSign * power);
                robot.leftRear.setPower(distanceSign * power);

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

        }
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

            if (curAngle * initAngle >= 0)
                referenceAngle = Math.abs(curAngle - initAngle);
            else {
                if (curAngle > 0)
                    referenceAngle = curAngle - initAngle;
                else
                    referenceAngle = (180 - initAngle) + (curAngle + 180);
            }

            calculatedPwr = power - referenceAngle * slope;

            if (calculatedPwr <= robot.MINIMUM_TURN_PWR)
                calculatedPwr = robot.MINIMUM_TURN_PWR;

            robot.rightRear.setPower(calculatedPwr);
            robot.leftRear.setPower(-calculatedPwr);

        }
    }

    public void unlatch(){
        //work on later
    }

    public void sample(){
        //work on later
    }

}

