package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    // Motors
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor extensionMotor;
    DcMotor rotationMotor;

    //Servos
    Servo leftClaw;
    Servo rightClaw;

    BNO055IMU imu;

    /* Teleop Constants */

    // servo constants (TBD, find accurate values through testing)
    final double LEFT_OPEN = 0.3;
    final double RIGHT_OPEN = 0.7;
    final double LEFT_CLOSE = 0.5;
    final double RIGHT_CLOSE = 0.5;

    // arm variables (TBD, find accurate values through testing)
    final double EXTENSION_POWER = 0.5;
    final int MAX_EXTENSION      = 5000;
    final int MIN_EXTENSION      = 0;
    final int MIN_ROTATION       = 0;
    final int MAX_ROTATION       = 5000;


    /* Autonomous Constants */

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

    //turning constants
    final double MINIMUM_TURN_PWR = 0.2;
}
