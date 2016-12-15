package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by agb on 12/8/2016.
 */

@TeleOp(name = "Example Teleop Arcade Mode Exponential")

public class ExampleTeleopArcadeModeExp extends OpMode {
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

        float xValue;
        float yValue;

        if (gamepad1.left_stick_x > 0) {
            xValue = gamepad1.left_stick_x * gamepad1.left_stick_x;
        } else {
            xValue = -gamepad1.left_stick_x * gamepad1.left_stick_x;
        }

        if (-gamepad1.left_stick_y > 0) {
            yValue = -gamepad1.left_stick_y * -gamepad1.left_stick_y;
        } else {
            yValue = gamepad1.left_stick_y * -gamepad1.left_stick_y;
        }

        float rightPower = yValue - xValue;
        float leftPower = yValue + xValue;

        rightPower = Range.clip(rightPower, -1, 1);
        leftPower = Range.clip(leftPower, -1, 1);

        RightFrontDrive.setPower(rightPower);
        RightRearDrive.setPower(rightPower);
        LeftFrontDrive.setPower(leftPower);
        LeftRearDrive.setPower(leftPower);
    }
}