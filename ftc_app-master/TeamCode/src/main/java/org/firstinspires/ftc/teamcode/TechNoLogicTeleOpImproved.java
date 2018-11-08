package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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
    DcMotor extensionMotor;
    DcMotor rotationMotor;

    Servo leftClaw;
    Servo rightClaw;

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

    @Override
    public void init() {

        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");

        leftClaw  = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");

        leftRear.setDirection(DcMotor.Direction.REVERSE);

        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        double bottomRotation = -gamepad2.right_stick_y;

        Range.clip(leftDrives, -0.75, 0.75);
        Range.clip(rightDrives, -0.75, 0.75);

        Range.clip(bottomRotation, -0.4, 0.4);
        
        // Set motor powers to joystick values
        leftRear.setPower(leftDrives);
        rightRear.setPower(rightDrives);

        /*
        if (rotationMotor.getCurrentPosition() <= MAX_ROTATION &&
                rotationMotor.getCurrentPosition() >= MIN_ROTATION) {
            rotationMotor.setPower(bottomRotation);
        } else
            extensionMotor.setPower(0.0);
*/

        rotationMotor.setPower(bottomRotation);

        if (gamepad2.a){
            leftClaw.setPosition(LEFT_OPEN);
            rightClaw.setPosition(RIGHT_OPEN);
        } else if (gamepad2.b){
            leftClaw.setPosition(LEFT_CLOSE);
            rightClaw.setPosition(RIGHT_CLOSE);
        }

        /*
        if (extensionMotor.getCurrentPosition() <= MAX_EXTENSION &&
                extensionMotor.getCurrentPosition() >= MIN_EXTENSION) {
            if (gamepad2.dpad_up)
                extensionMotor.setPower(EXTENSION_POWER);
            else if (gamepad2.dpad_down)
                extensionMotor.setPower(-EXTENSION_POWER);
            else
                extensionMotor.setPower(0.0);
        } else
            extensionMotor.setPower(0.0);
*/

        if (gamepad2.dpad_up)
            extensionMotor.setPower(EXTENSION_POWER);
        else if (gamepad2.dpad_down)
            extensionMotor.setPower(-EXTENSION_POWER);
        else
            extensionMotor.setPower(0.0);

        // Telemetry output for driver assistance
        telemetry.addData(">", "Press Stop to end program." );
        telemetry.addData(">", "linear slide encoder value: " + extensionMotor.getCurrentPosition());
        telemetry.addData(">", "rotation encoder value: " + rotationMotor.getCurrentPosition());
        telemetry.update();
    }
}

