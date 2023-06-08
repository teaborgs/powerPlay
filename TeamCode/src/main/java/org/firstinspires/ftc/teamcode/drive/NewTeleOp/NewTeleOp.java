package org.firstinspires.ftc.teamcode.drive.NewTeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


enum RotationPos
{

}


@TeleOp(group = "TeleOp")
public class NewTeleOp extends LinearOpMode
{
    SampleMecanumDrive mecanumDrive;
    Gamepad gamepadWheels;
    Gamepad gamepadArm;

    DcMotorEx wormMotor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Init();

        waitForStart();

        while (!isStopRequested())
            Run();
    }

    private void Init()
    {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

        wormMotor = hardwareMap.get(DcMotorEx.class, "wormMotor");

        gamepadWheels = gamepad1;
        gamepadArm = gamepad2;
    }

    private void Run()
    {
        HandleMovement();
        HandleArm();
    }

    private void HandleMovement()
    {
        Pose2d input = new Pose2d(gamepadWheels.left_stick_x, gamepadWheels.left_stick_y);
        input = ProcessInput(input);

        mecanumDrive.setWeightedDrivePower(input);
    }

    private void HandleArm()
    {
        // rotatie brat
        if (gamepadArm.dpad_down)
        {
            // fata

        }

        // nivel brat


    }

    private Pose2d ProcessInput(Pose2d input)
    {
        return new Pose2d(Fn(input.getX()), Fn(input.getY()));
    }

    private double Fn(double val)
    {
        double sign = Math.abs(val) / val;
        return val * val * sign;
    }
}