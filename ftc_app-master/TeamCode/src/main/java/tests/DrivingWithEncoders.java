package tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DrivingWithEncoders extends LinearOpMode {

    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;

    // counts per inch (CPI) calculation
    final double COUNTS_PER_MOTOR_REV = 1440.;    // eg: Andymark Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1.;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_INCHES = 4.;     // For figuring circumference
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //drive function constants
    final double MINIMUM_DRIVE_PWR = 0.15;
    final double DECELERATION_THRESHOLD = 7.0;
    final double ACC_CONSTANT = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

    }

    // power should always be POSITIVE
    // distance should be in INCHES
    // positive distance indicates driving forward
    // negative distance indicates driving backwards
    private void encoderDrive(double distance, double maxPwr) {

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double brakeDist    = (Math.abs(distance) - DECELERATION_THRESHOLD) * COUNTS_PER_INCH;
        double accThreshold = Math.abs(distance) * ACC_CONSTANT * COUNTS_PER_INCH;

        double initPosR = rightFront.getCurrentPosition();
        double initPosL = leftFront.getCurrentPosition();

        double targetPos    = distance * COUNTS_PER_INCH;
        double distanceSign = Math.signum(distance);

        double currentRobotPos = 0.;

        double power = maxPwr;

        // slopes for proportional speed increase/decrease
        double decSlope = (maxPwr - MINIMUM_DRIVE_PWR) / (DECELERATION_THRESHOLD);
        double accSlope = (maxPwr - MINIMUM_DRIVE_PWR) / (accThreshold);

        while (Math.abs(currentRobotPos) < Math.abs(targetPos)){

            double curPosR = rightFront.getCurrentPosition() - initPosR;
            double curPosL = leftFront.getCurrentPosition() - initPosL;

            currentRobotPos = (curPosR + curPosL) / 2;


            // calculating points on trapezoidal profile graph
            if (Math.abs(currentRobotPos) < accThreshold)
                power = currentRobotPos * accSlope;
            else if (Math.abs(currentRobotPos) > brakeDist)
                power = maxPwr - (Math.abs(currentRobotPos) - brakeDist) * decSlope;
            else
                power = maxPwr;

            if (power < MINIMUM_DRIVE_PWR)
                power = MINIMUM_DRIVE_PWR;

            rightFront.setPower(distanceSign * power);
            rightRear.setPower(distanceSign * power);
            leftFront.setPower(distanceSign * power);
            leftRear.setPower(distanceSign * power);

        }

        rightFront.setPower(0.);
        rightRear.setPower(0.);
        leftFront.setPower(0.);
        leftRear.setPower(0.);

    }

}
