package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by agb on 12/15/2016.
 */

@TeleOp(name = "Toggle driving with gatherer and launcher")

public class ToggleDriveGathererLauncher extends OpMode
{

    DcMotor RightFrontDrive;
    DcMotor LeftFrontDrive;
    DcMotor RightRearDrive;
    DcMotor LeftRearDrive;
    DcMotor GathererMotor;
    DcMotor LauncherMotor;

    @Override
    public void init() {
        RightFrontDrive = hardwareMap.dcMotor.get("right front drive");
        LeftFrontDrive = hardwareMap.dcMotor.get("left front drive");
        RightRearDrive = hardwareMap.dcMotor.get("right rear drive");
        LeftRearDrive = hardwareMap.dcMotor.get("left rear drive");
        GathererMotor = hardwareMap.dcMotor.get("gatherer motor");
        LauncherMotor = hardwareMap.dcMotor.get("launcher motor");

        LeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        GathererMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    boolean arcadeDriveStyle = false;
    boolean driveStyleWasPressed = false;

    @Override
    public void loop()
    {
        if(gamepad1.start && gamepad1.back)
        {
            if(!driveStyleWasPressed) // if the buttons weren't pressed last time
            {
                if (arcadeDriveStyle) {arcadeDriveStyle = false;} // if arcadeDriveStyle is true, make it false
                else if (!arcadeDriveStyle) {arcadeDriveStyle = true;} // if arcadeDriveStyle is false, make it true
                driveStyleWasPressed = true;
            }
        }
        else // if the buttons aren't pressed
        {
            driveStyleWasPressed = false;
        }

        if(arcadeDriveStyle)
        {
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

        if(!arcadeDriveStyle)
        {
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

        double gathererMotorValue = 0;

        if (gamepad1.right_trigger != 0){
            gathererMotorValue = gamepad1.right_trigger;
        }
        else if(gamepad1.left_trigger != 0) {
            gathererMotorValue = -gamepad1.left_trigger;
        }

        GathererMotor.setPower(gathererMotorValue);

        if(gamepad1.right_bumper){LauncherMotor.setPower(1);}
        else if(gamepad1.left_bumper){LauncherMotor.setPower(-1);}
        else {LauncherMotor.setPower(0);}
    }
}
