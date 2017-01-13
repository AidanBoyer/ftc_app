package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by agb on 12/8/2016.
 */

@TeleOp(name = "Example Teleop Tank Mode Exponential")
@Disabled

public class ExampleTeleopTankModeExp extends OpMode {

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
        double rightStickVal = -gamepad1.right_stick_y;
        double leftStickVal = -gamepad1.left_stick_y;
        double rightSquaredVal = rightStickVal * rightStickVal;
        double leftSquaredVal = leftStickVal * leftStickVal;

        if(rightStickVal < 0) {
            RightFrontDrive.setPower(-rightSquaredVal);
            RightRearDrive.setPower(-rightSquaredVal);
        } else {
            RightFrontDrive.setPower(rightSquaredVal);
            RightRearDrive.setPower(rightSquaredVal);
        }
        if(leftStickVal < 0) {
            LeftFrontDrive.setPower(-leftSquaredVal);
            LeftRearDrive.setPower(-leftSquaredVal);
        } else {
            LeftFrontDrive.setPower(leftSquaredVal);
            LeftRearDrive.setPower(leftSquaredVal);
        }
    }
}
