package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by agb on 1/12/2017.
 */

@Autonomous(name = "Fire launcher")

public class fireLauncher extends LinearOpMode {
    DcMotor RightFrontDrive;
    DcMotor LeftFrontDrive;
    DcMotor RightRearDrive;
    DcMotor LeftRearDrive;
    DcMotor GathererMotor;
    DcMotor LauncherMotor;

    public void fireLauncher()
    {
        int ticksPerLauncherRev = (int)(1478.4 * 1.4);
        LauncherMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LauncherMotor.setTargetPosition(LauncherMotor.getCurrentPosition() + ticksPerLauncherRev);
        LauncherMotor.setPower(1);
        while(opModeIsActive() && LauncherMotor.isBusy()) {}
        LauncherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        RightFrontDrive = hardwareMap.dcMotor.get("right front drive");
        LeftFrontDrive = hardwareMap.dcMotor.get("left front drive");
        RightRearDrive = hardwareMap.dcMotor.get("right rear drive");
        LeftRearDrive = hardwareMap.dcMotor.get("left rear drive");
        GathererMotor = hardwareMap.dcMotor.get("gatherer motor");
        LauncherMotor = hardwareMap.dcMotor.get("launcher motor");

        LeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        GathererMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        fireLauncher();
    }
}
