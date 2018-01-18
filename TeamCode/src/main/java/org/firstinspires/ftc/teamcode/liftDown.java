package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by agb on 1/9/2018.
 */

@Autonomous(name = "lift to 0")

public class liftDown extends LinearOpMode {

    DcMotor LiftMotor;
    ServoController ServoController1;

    @Override
    public void runOpMode() throws InterruptedException {

        LiftMotor = hardwareMap.dcMotor.get("lift motor");
        ServoController1 = hardwareMap.servoController.get("servo controller 1");
        ServoController1.pwmDisable();

        waitForStart();

        LiftMotor.setTargetPosition(0);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setPower(-1);

        while(opModeIsActive() && LiftMotor.isBusy()) {}

    }
}
