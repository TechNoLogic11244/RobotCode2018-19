package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class FindServoPosition extends OpMode {

    Servo rightClaw;
    Servo leftClaw;

    double rightPos = 0.5;
    double leftPos = 0.5;

    boolean aPressed = false;
    boolean bPressed = false;
    boolean xPressed = false;
    boolean yPressed = false;

    @Override
    public void init() {

        rightClaw = hardwareMap.servo.get("rightClaw");
        leftClaw = hardwareMap.servo.get("leftClaw");

        rightClaw.setPosition(rightPos);
        leftClaw.setPosition(leftPos);

    }

    @Override
    public void loop() {

        if (gamepad2.a){
            if (!aPressed) {
                rightPos++;
                aPressed = true;
            }
        } else
            aPressed = false;

        if (gamepad2.b) {
            if (!bPressed) {
                rightPos--;
                bPressed = true;
            }
        } else
            bPressed = false;

        if (gamepad2.x) {
            if (!xPressed) {
                leftPos++;
                xPressed = true;
            }
        } else
            xPressed = false;

        if (gamepad2.y) {
            if (!yPressed) {
                leftPos--;
                yPressed = true;
            }
        } else
            yPressed = false;

        rightClaw.setPosition(rightPos);
        leftClaw.setPosition(leftPos);

        telemetry.addData(">", "left pos = " + leftPos);
        telemetry.addData(">", "right pos = " + rightPos);

    }
}
