package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by agb on 12/15/2016.
 */

@TeleOp(name = "Toggle driving with both gamepads")

public class ToggleDriveBothGamepads extends OpMode
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
    boolean driveReversed = false;
    boolean wasReverseButton = false;


    @Override
    public void loop()
    {
        double rightFrontDriveValue = 0;
        double rightRearDriveValue = 0;
        double leftFrontDriveValue = 0;
        double leftRearDriveValue = 0;

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

            rightFrontDriveValue = rightPower;
            rightRearDriveValue = rightPower;
            leftFrontDriveValue = leftPower;
            leftRearDriveValue = leftPower;
        }

        if(!arcadeDriveStyle)
        {
            double rightStickVal = -gamepad1.right_stick_y;
            double leftStickVal = -gamepad1.left_stick_y;
            double rightSquaredVal = rightStickVal * rightStickVal;
            double leftSquaredVal = leftStickVal * leftStickVal;

            if(rightStickVal < 0) {
                rightFrontDriveValue = -rightSquaredVal;
                rightRearDriveValue = -rightSquaredVal;
            } else {
                rightFrontDriveValue = rightSquaredVal;
                rightRearDriveValue = rightSquaredVal;
            }
            if(leftStickVal < 0) {
                leftFrontDriveValue = -leftSquaredVal;
                leftRearDriveValue = -leftSquaredVal;
            } else {
                leftFrontDriveValue = leftSquaredVal;
                leftRearDriveValue = leftSquaredVal;
            }
        }

        if (gamepad1.right_trigger != 0)
        {
            rightFrontDriveValue = gamepad1.right_trigger;
            rightRearDriveValue = gamepad1.right_trigger;
            leftFrontDriveValue = gamepad1.right_trigger;
            leftRearDriveValue = gamepad1.right_trigger;
        }
        else if (gamepad1.left_trigger != 0)
        {
            rightFrontDriveValue = -gamepad1.left_trigger;
            rightRearDriveValue = -gamepad1.left_trigger;
            leftFrontDriveValue = -gamepad1.left_trigger;
            leftRearDriveValue = -gamepad1.left_trigger;
        }
        double gathererMotorValue = 0;

        if (gamepad2.right_trigger != 0){
            gathererMotorValue = gamepad2.right_trigger;
        }
        else if(gamepad2.left_trigger != 0) {
            gathererMotorValue = -gamepad2.left_trigger;
        }

        GathererMotor.setPower(gathererMotorValue);

        if(gamepad2.right_bumper){LauncherMotor.setPower(1);}
        else if(gamepad2.left_bumper){LauncherMotor.setPower(-1);}
        else {LauncherMotor.setPower(0);}

        if(gamepad1.left_bumper)
        {
            rightFrontDriveValue = rightFrontDriveValue / 2;
            rightRearDriveValue = rightRearDriveValue / 2;
            leftFrontDriveValue = leftFrontDriveValue / 2;
            leftRearDriveValue = leftRearDriveValue / 2;
        }

        if(gamepad1.right_bumper)
        {
            if(!wasReverseButton)
            {
                if(!driveReversed) {driveReversed = true;}
                else {driveReversed = false;}
            }
            wasReverseButton = true;
        }
        else
        {
            wasReverseButton = false;
        }

        if(driveReversed)
        {
            double RR = rightRearDriveValue;
            double LR = leftRearDriveValue;
            double RF = rightFrontDriveValue;
            double LF = leftFrontDriveValue;

            leftRearDriveValue = RR * -1;
            rightRearDriveValue = LR * -1;
            leftFrontDriveValue = RF * -1;
            rightFrontDriveValue = LF * -1;
        }

        RightFrontDrive.setPower(rightFrontDriveValue);
        RightRearDrive.setPower(rightRearDriveValue);
        LeftFrontDrive.setPower(leftFrontDriveValue);
        LeftRearDrive.setPower(leftRearDriveValue);

    }
}
