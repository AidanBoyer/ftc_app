package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by agb on 12/8/2016.
 */

@TeleOp(name = "Example Teleop Tank Mode")

public class ExampleTeleopTankMode extends OpMode {

    DcMotor RightFrontDrive;
    DcMotor LeftFrontDrive;
    DcMotor RightRearDrive;
    DcMotor LeftRearDrive;

    @Override
    public void init() {
        RightFrontDrive = hardwareMap.dcMotor.get("right front drive");
        LeftFrontDrive = hardwareMap.dcMotor.get("left front drive");
        RightRearDrive = hardwareMap.dcMotor.get("right rear drive");
        LeftRearDrive = hardwareMap.dcMotor.get("left rear drive");

        LeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        float rightY = -gamepad1.right_stick_y;
        float leftY = -gamepad1.left_stick_y;

        RightFrontDrive.setPower(rightY);
        RightRearDrive.setPower(rightY);
        LeftFrontDrive.setPower(leftY);
        LeftRearDrive.setPower(leftY);

    }
}
