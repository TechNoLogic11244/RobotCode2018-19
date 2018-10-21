package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.tree.DCTree;

/**
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "left_drive" as is found on a pushbot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "TechNoLogic TeleOp Improved", group = "teleop")
//@Disabled
public class TechNoLogicTeleOpImproved extends OpMode{

    // Define class members
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor topArm;
    DcMotor bottomArm;

    Servo leftClaw;
    Servo rightClaw;

    // put llamas as the variable name :)
    final double LEFT_OPEN = 0.3;
    final double RIGHT_OPEN = 0.7;
    final double LEFT_CLOSE = 0.5;
    final double RIGHT_CLOSE = 0.5;

    @Override
    public void init() {

        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        topArm = hardwareMap.get(DcMotor.class, "topArm");
        bottomArm = hardwareMap.get(DcMotor.class, "bottomArm");

        leftClaw  = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");

        leftRear.setDirection(DcMotor.Direction.REVERSE);

        // Can use this line, but can also use opposites (set one to 0, the other to 1)
        //rightClaw.setDirection(Servo.Direction.REVERSE);

        // Wait for the start button
        telemetry.addData(">", "Press Start" );
        telemetry.update();
    }

    @Override
    public void loop() {

        // Variables for joystick values
        double leftDrives  = -gamepad1.left_stick_y;
        double rightDrives = -gamepad1.right_stick_y;

        double linearSlide = -gamepad2.left_stick_y;
        double bottomRotation = -gamepad2.right_stick_y;

        // Set motor powers to joystick values
        leftRear.setPower(leftDrives);
        rightRear.setPower(rightDrives);

        topArm.setPower(linearSlide);
        bottomArm.setPower(bottomRotation);

        if (gamepad2.a){
            leftClaw.setPosition(LEFT_OPEN);
            rightClaw.setPosition(RIGHT_OPEN);
        } else if (gamepad2.b){
            leftClaw.setPosition(LEFT_CLOSE);
            rightClaw.setPosition(RIGHT_CLOSE);
        }

        // Telemetry output for driver assistance
        telemetry.addData(">", "Press Stop to end program." );
        telemetry.update();
    }
}

