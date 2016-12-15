package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by agb on 12/1/2016.
 */

@TeleOp(name="drive test")

public class autonomous_drive_test extends OpMode {

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

        RightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {



        /*RightFrontDrive.setPower(50);

        wait(1000);

        RightFrontDrive.setPower(0);
        RightRearDrive.setPower(50);

        wait(1000);

        RightRearDrive.setPower(0);
        LeftFrontDrive.setPower(50);

        wait(1000);

        LeftFrontDrive.setPower(0);
        LeftRearDrive.setPower(50);

        wait(1000);*/

        RightFrontDrive.setPower(100);
        LeftFrontDrive.setPower(100);
        RightRearDrive.setPower(100);
        LeftRearDrive.setPower(100);
    }
}
