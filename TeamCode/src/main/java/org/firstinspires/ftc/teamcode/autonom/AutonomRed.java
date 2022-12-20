package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.AutoPosition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutonomRed{
    Autonom auto;
    TrajectorySequence deliverPreload,catchCone1,deliverCone12,catchCone2,catchCone3,catchCone4,catchCone5,deliverCone345;

    public AutonomRed(Autonom auto){
        this.auto = auto;
        initializeTrajectories();
    }

    private void initializeTrajectories(){
        deliverPreload = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .strafeRight(55)
                .back(11)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        catchCone1 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverPreload.end())
                .forward(29)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE5))
                .build();
        deliverCone12 = auto.mecanumDrive
                .trajectorySequenceBuilder(catchCone1.end())
                .back(29)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        catchCone2 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverCone12.end())
                .strafeRight(0.8)
                .forward(30)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE4))
                .build();
        deliverCone345 = auto.mecanumDrive
                .trajectorySequenceBuilder(catchCone1.end())
                .back(29)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        catchCone3 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverCone345.end())
                .strafeRight(0.7)
                .forward(29.5)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE3))
                .build();
        catchCone4 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverCone345.end())
                .forward(30)
                .addTemporalMarker( 0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE2))
                .build();
        catchCone5 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverCone345.end())
                .forward(30.5)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
    }

    public void runAuto(){
        auto.mecanumDrive.followTrajectorySequence(deliverPreload);
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(200);
        auto.mecanumDrive.followTrajectorySequence(catchCone1);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone12);
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.mecanumDrive.followTrajectorySequence(catchCone2);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone12);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.mecanumDrive.followTrajectorySequence(catchCone3);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone345);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.mecanumDrive.followTrajectorySequence(catchCone4);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone345);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.mecanumDrive.followTrajectorySequence(catchCone5);
        auto.sleep(150);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone345);
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(30000);
    }
}
