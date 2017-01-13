package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by agb on 1/11/2017.
 */

@TeleOp(name = "Color sensor test")
@Disabled
public class colorSensorOutput extends OpMode {

    ModernRoboticsI2cColorSensor ColorSensor;

    @Override
    public void init() {
        ColorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "MR color sensor");
        ColorSensor.enableLed(false);
    }

    @Override
    public void loop() {
        //telemetry.addData("Color sensor value ", ColorSensor.);
    }
}
