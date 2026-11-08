package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class TurretMechanism {

    private DcMotorEx turret;

    private static final double GOBILDA_TICKS = 1535.24;


    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );



    }

    public double update(Pose2D robotPos) {
        double robotX = robotPos.getX(DistanceUnit.INCH);
        double robotY = robotPos.getY(DistanceUnit.INCH);

        double robotHeading = robotPos.getHeading(AngleUnit.DEGREES);

        // Calculate direction from robot to target
        double changeX = 48 - robotX;
        double changeY = 60 - robotY;

        double fieldAngle = -Math.toDegrees(
                Math.atan2(changeY, changeX)
        );

        // Calculate desired turret angle relative to robot
        double turretAngle = fieldAngle + robotHeading;

        // Normalize angle
        turretAngle = ((turretAngle + 180) % 360 + 360) % 360 - 180;

        // Convert to encoder ticks
        double targetTicks = turretAngle / 360.0 * GOBILDA_TICKS;

        // Find the closest equivalent encoder target
        int currentTicks = turret.getCurrentPosition();

        targetTicks += Math.round(
                (currentTicks - targetTicks) / GOBILDA_TICKS
        ) * GOBILDA_TICKS;

        turret.setTargetPosition((int) targetTicks);
        turret.setPower(1.0);

        return turretAngle;
    }
}