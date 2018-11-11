package tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousBase;

@Autonomous(name = "turn left test")
public class TurnLeftTest extends AutonomousBase {

    @Override
    public void runOpMode() {

        initialization();

        waitForStart();

        turnLeft(20, 0.6);

    }

/*
    int angle = 180;
    boolean ifTurnRequested = true;
    boolean ifButtonPressed = false;

    @Override
    public void runOpMode() {

        initialization();

        waitForStart();

        while (opModeIsActive()) {

            if (!ifButtonPressed) {
                if (gamepad1.dpad_up) {
                    angle += 10;
                    ifTurnRequested = true;
                    ifButtonPressed = true;
                }
                if (gamepad1.dpad_down) {
                    angle -= 10;
                    ifTurnRequested = true;
                    ifButtonPressed = true;
                } else {
                    ifTurnRequested = false;
                    ifButtonPressed = false;
                }
            }

            telemetry.addData(">", "turning angle = " + angle);
            telemetry.update();

            if (ifTurnRequested) {
                turnLeft(angle, 0.3);
                ifTurnRequested = false;
            }

        }

    }
*/
}
//hi there yimmey wassup yimmey llama man GUCCI GANG GUCCI GANG GUCCI GANG [insert default dance here])