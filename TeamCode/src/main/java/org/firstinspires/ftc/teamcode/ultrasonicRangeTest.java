package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by agb on 1/13/2017.
 */

@TeleOp(name = "Ultrasonic test")
@Disabled

public class ultrasonicRangeTest extends OpMode {

    UltrasonicSensor ultrasonicRangeSensor;

    @Override
    public void init() {
        ultrasonicRangeSensor = hardwareMap.ultrasonicSensor.get("ultrasonic range");
    }

    @Override
    public void loop() {
        telemetry.addData("Ultrasonic level: ", ultrasonicRangeSensor.getUltrasonicLevel());
        telemetry.update();
    }
}
