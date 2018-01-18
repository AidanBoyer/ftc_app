package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CompassSensor;

/**
 * Created by agb on 12/31/2017.
 */

@Autonomous(name = "tilt sensor test")
@Disabled

public class compassTest extends OpMode
{
    ModernRoboticsI2cCompassSensor tiltSensor;

    @Override
    public void init() {
        tiltSensor = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass sensor");
    }

    @Override
    public void loop() {
        telemetry.addData("Value:", tiltSensor.readCommand());
        telemetry.update();
    }
}
