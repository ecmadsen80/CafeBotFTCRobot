package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class TurretMechanism {

    private DcMotorEx turret;

    private static final double GOBILDA_TICKS = 751.8;


    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );


    }

    public void update(Pose2D robotPos) {
        double robotX = robotPos.getX(DistanceUnit.INCH);
        double robotY = robotPos.getY(DistanceUnit.INCH);
        double changeX = robotX - 48;
        double changeY = robotY - 60;
        double degrees = Math.toDegrees(Math.atan2(changeY, changeX));

        turret.setTargetPosition((int)(degrees / 360 * GOBILDA_TICKS));
    }
}