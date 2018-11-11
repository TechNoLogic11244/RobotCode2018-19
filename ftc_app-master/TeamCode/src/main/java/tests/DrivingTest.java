package tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousBase;

@Autonomous(name = "driving test")
public class DrivingTest extends AutonomousBase {


    @Override
    public void runOpMode() {

        initialization();

        waitForStart();

        encoderDrive(24, 0.5);

    }

    /*
    int driveDist = 24;
    boolean ifDriveRequested = true;
    boolean ifButtonPressed = false;

    @Override
    public void runOpMode() {

        initialization();

        waitForStart();

        while (opModeIsActive()) {

            if (!ifButtonPressed) {
                if (gamepad1.dpad_up) {
                    driveDist += 2;
                    ifDriveRequested = true;
                    ifButtonPressed = true;
                }
                if (gamepad1.dpad_down) {
                    driveDist -= 2;
                    ifDriveRequested = true;
                    ifButtonPressed = true;
                } else {
                    ifDriveRequested = false;
                    ifButtonPressed = false;
                }
            }

            telemetry.addData(">", "drive dist = " + driveDist);
            telemetry.update();

            if (ifDriveRequested) {
                encoderDrive(driveDist, 0.3);
                ifDriveRequested = false;
            }

        }

    }*/

}
