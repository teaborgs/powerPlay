package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.ElectricTeleOp;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutonomRed{

    Autonom auto;
    TrajectorySequence deliverPreload,rotate,catchCone1,deliverCone1,catchCone2,deliverCone2;

    public AutonomRed(Autonom auto){
        this.auto = auto;
        initializeTrajectories();
    }

    /*private void initializeTrajectories() {
        deliverPreload = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .strafeRight(22)
                .back(11)
                .build();
        rotate = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverPreload.end())
                .strafeLeft(3)
                .addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.plateMotor,2400))
                .back(43)
                .build();
        deliverCone1 = auto.mecanumDrive
                .trajectorySequenceBuilder(rotate.end())
                .strafeLeft(34)
                .addTemporalMarker(.05, () -> AutoUtil.liftPosition(auto.liftMotor,-2650))
                .addTemporalMarker(.2, () -> AutoUtil.platePosition(auto.plateMotor,3400))
                //.back(1)
                .build();
        catchCone2 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverCone1.end())
                .forward(2)
                .strafeRight(34)
                .back(1)
                .addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.plateMotor,2400))
                .addTemporalMarker(.4, () -> AutoUtil.liftPosition(auto.liftMotor,-280))
                .build();
        deliverCone2 = auto.mecanumDrive
                .trajectorySequenceBuilder(catchCone2.end())
                .strafeLeft(34.5)
                //.back(1)
                .addTemporalMarker(.1, () -> AutoUtil.liftPosition(auto.liftMotor,-2650))
                .addTemporalMarker(.2, () -> AutoUtil.platePosition(auto.plateMotor,3400))
                .build();
    }*/

    private void initializeTrajectories(){
        deliverPreload = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .strafeRight(59)
                .strafeLeft(6)
                .back(6)
                .addTemporalMarker(.05, () -> AutoUtil.liftPosition(auto.liftMotor,-2650))
                .build();
        catchCone1 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverPreload.end())
                .strafeLeft(2)
                .forward(23)
                .addTemporalMarker(.05, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.2, () -> AutoUtil.liftPosition(auto.liftMotor,-280))
                .build();
    }

    /*public void runAuto(){
        auto.AutoUtil.liftPosition(auto.liftMotor,-100);
        auto.mecanumDrive.followTrajectorySequence(deliverPreload);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(300);
        auto.AutoUtil.liftPosition(auto.liftMotor,-975);
        auto.mecanumDrive.followTrajectorySequence(rotate);
        auto.AutoUtil.liftPosition(auto.liftMotor,-340);
        auto.sleep(600);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(600);
        auto.mecanumDrive.followTrajectorySequence(deliverCone1);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.mecanumDrive.followTrajectorySequence(catchCone2);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(400);
        auto.mecanumDrive.followTrajectorySequence(deliverCone2);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
    }*/

    public void runAuto(){
        auto.AutoUtil.liftPosition(auto.liftMotor,-100);
        auto.mecanumDrive.followTrajectorySequence(deliverPreload);
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.mecanumDrive.followTrajectorySequence(catchCone1);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);

    }
}
