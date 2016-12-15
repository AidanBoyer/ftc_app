package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by agb on 12/15/2016.
 */

@TeleOp(name = "Gatherer test")

public class gathererTest extends OpMode {

    DcMotor RightFrontDrive;
    DcMotor LeftFrontDrive;
    DcMotor RightRearDrive;
    DcMotor LeftRearDrive;
    DcMotor GathererMotor;

    @Override
    public void init() {
        RightFrontDrive = hardwareMap.dcMotor.get("right front drive");
        LeftFrontDrive = hardwareMap.dcMotor.get("left front drive");
        RightRearDrive = hardwareMap.dcMotor.get("right rear drive");
        LeftRearDrive = hardwareMap.dcMotor.get("left rear drive");
        GathererMotor = hardwareMap.dcMotor.get("gatherer motor");

        LeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        GathererMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double gathererMotorValue = 0;

        if (gamepad1.right_trigger != 0){
            gathererMotorValue = gamepad1.right_trigger;
        }
        else if(gamepad1.left_trigger != 0) {
            gathererMotorValue = -gamepad1.left_trigger;
        }

        GathererMotor.setPower(gathererMotorValue);

    }
}
