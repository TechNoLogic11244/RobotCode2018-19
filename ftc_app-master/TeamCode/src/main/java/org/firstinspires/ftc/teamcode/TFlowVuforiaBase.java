package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


public class TFlowVuforiaBase {

    private static final String VUFORIA_KEY = "AcHjh/L/////AAABmcefxvFABE4egC7kE6SDokyA19FitpO9VTpAUQ/GPclYxIQgGBA0Sr5gYRvVcRDvCx1CsTjPNA9Sf8Kyg9DyYMsIOfDxHMB90T7EDD6hf2IOO/8H3Q838aFilXo1vbpfSjJCtmK+0YRZYiPzNMsELmk4iC8FAfJdI7xPjIVQBtZtnB5b8V2g19HDUHzs36JuiwuVUScojAEKJh/cLmD/XNo74C2HXWO0DiVU/i1fnJdSf4At0Vu0HeAEPer2hQcPZFkuRyHElmsSA+Uj9kXlNCCb+9Lv3g8RFLJzNANPwDoMrxkCppQhuY4LkDoQo+HHfvq7RcKW11eZaipdYeoK95dj2GUMWk6QMNjnm2vVVPma";

    public VuforiaLocalizer vuforia;

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public TFObjectDetector tfod;

    public void initTFlowAndVuforia(){

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }

    }

    private void initTfod() {

        // can use this to display camera feed on phone screen
        //TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(R.id.cameraMonitorViewId);

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}
