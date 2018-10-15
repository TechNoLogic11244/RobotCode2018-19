package tests;

import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Sensor Tests", group = "sensors")
public class SensorTests extends OpMode {

    // sensor variable declarations
    TouchSensor touch;
    ColorSensor color;

    @Override
    public void init() {

        touch = hardwareMap.touchSensor.get("touch");
        color = hardwareMap.colorSensor.get("color");

        color.enableLed(true);

    }

    @Override
    public void loop() {
 1
        // check whether touch sensor is pressed or not pressed,
        // then display telemetry feedback based on sensor value
        if (touch.isPressed())
            telemetry.addData("touch", "touch sensor pressed");
        else
            telemetry.addData("touch", "touch sensor not pressed");

        detectColor();

    }

    // displays telemetry for rgb values detected by color sensor
    private void detectColor() {
        telemetry.addData("color", "red value: " + color.red());
        telemetry.addData("color", "blue value: " + color.blue());
        telemetry.addData("color", "green value:" + color.green());
    }

}
