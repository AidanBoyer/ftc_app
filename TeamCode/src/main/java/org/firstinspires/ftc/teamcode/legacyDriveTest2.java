package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.matrix.MatrixDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by agb on 9/12/2016.
 */
//@TeleOp(name="Template: Linear OpMode", group="Linear Opmode")

//@TeleOp(name="legacy drive test - nor")
public class legacyDriveTest2 extends OpMode
{
    private MatrixDcMotorController matrixMotorController1 = null;
    private MatrixDcMotorController matrixMotorController2 = null;

    private DcMotor RightFrontDrive;
    private DcMotor LeftFrontDrive;
    private DcMotor RightRearDrive;
    private DcMotor LeftRearDrive;

    /*double scaledJoyDrive(double i_joyY)
    {
        double outputVariable;
        outputVariable = i_joyY * i_joyY;
        if (i_joyY < 0)
        {
            outputVariable = outputVariable * -1;
        }
        return outputVariable;
    }*/

    @Override
    public void init() {

        matrixMotorController1 = (MatrixDcMotorController)hardwareMap.dcMotorController.get("matrix controller 1");

        RightFrontDrive = hardwareMap.dcMotor.get("right front drive");
        LeftFrontDrive = hardwareMap.dcMotor.get("left front drive");
        RightRearDrive = hardwareMap.dcMotor.get("right rear drive");
        LeftRearDrive = hardwareMap.dcMotor.get("left rear drive");

        LeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //boolean driveReversed = false;
    //boolean wasReverseButton = false;

    @Override
    public void loop()
    {


        double LeftRearDriveValue;
        double RightRearDriveValue;
        double LeftFrontDriveValue;
        double RightFrontDriveValue;

        LeftRearDriveValue = gamepad1.left_stick_y * -1;
        RightRearDriveValue = gamepad1.right_stick_y * -1;
        LeftFrontDriveValue = gamepad1.left_stick_y * -1;
        RightFrontDriveValue = gamepad1.right_stick_y * -1;


        if(gamepad1.dpad_down)
        {
            LeftRearDriveValue =  -100;
            RightRearDriveValue = -100;
            LeftFrontDriveValue =  -100;
            RightFrontDriveValue = -100;
        }
        else if(gamepad1.dpad_up)
        {
            LeftRearDriveValue =  100;
            RightRearDriveValue = 100;
            LeftFrontDriveValue =  100;
            RightFrontDriveValue = 100;
        }
        else if(gamepad1.dpad_left)
        {
            LeftRearDriveValue =  -50;
            RightRearDriveValue = 50;
            LeftFrontDriveValue =  -50;
            RightFrontDriveValue = 50;
        }
        else if(gamepad1.dpad_right)
        {
            LeftRearDriveValue =  50;
            RightRearDriveValue = -50;
            LeftFrontDriveValue =  50;
            RightFrontDriveValue = -50;
        }

        if(!gamepad1.left_bumper) // if the left bumper is NOT pressed, cut values in half
        {

            LeftRearDriveValue = LeftRearDriveValue / 2;
            RightRearDriveValue = RightRearDriveValue / 2;
            LeftFrontDriveValue = LeftFrontDriveValue / 2;
            RightFrontDriveValue = RightFrontDriveValue / 2;
        }

       /* if(gamepad1.right_bumper)
        {
            if(!wasReverseButton) // only do the reversing if the button was NOT already pressed. eg. don't reverse back and forth really fast with button hold
            {
                if(!driveReversed) {driveReversed = true;} // if the drive is not reversed, reverse it
                else {driveReversed = false;} // if the drive is reversed, unreverse it
            }
            wasReverseButton = true;
        }
        else
        {
            wasReverseButton = false;
        }

        if(driveReversed)
        {
            double RR = RightRearDriveValue;
            double LR = LeftRearDriveValue;
            double RF = RightFrontDriveValue;
            double LF = LeftFrontDriveValue;

            LeftRearDriveValue = RR * -1;
            RightRearDriveValue = LR * -1;
            LeftFrontDriveValue = RF * -1;
            RightFrontDriveValue = LF * -1;
        }
        */

        LeftRearDrive.setPower(LeftRearDriveValue);
        RightRearDrive.setPower(RightRearDriveValue);
        LeftFrontDrive.setPower(LeftFrontDriveValue);
        RightFrontDrive.setPower(RightFrontDriveValue);
    }
}
