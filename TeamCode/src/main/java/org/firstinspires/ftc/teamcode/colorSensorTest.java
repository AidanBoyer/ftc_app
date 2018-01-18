package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;


/**
 * Created by agb on 1/1/2018.
 */

public class colorSensorTest extends OpMode
{
    ColorSensor jewelColorSensor;

    @Override
    public void init() {
        jewelColorSensor = hardwareMap.colorSensor.get("jewel color sensor");
        jewelColorSensor.enableLed(true);
    }

    @Override
    public void loop() {
        //telemetry.addData("Color number:", jewelColorSensor.
    }
}
