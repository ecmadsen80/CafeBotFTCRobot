package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DashboardVisualizer {
    private final FtcDashboard dashboard;
    private final GoBildaPinpointDriver odo;

    public DashboardVisualizer(GoBildaPinpointDriver odo) {
        this.odo = odo;
        this.dashboard = FtcDashboard.getInstance();
    }

    public void update() {
        // Ensure odometry is updated
        odo.update();
        Pose2D pos = odo.getPosition();

        double robotX = pos.getX(DistanceUnit.INCH);
        double robotY = pos.getY(DistanceUnit.INCH);
        double robotHeading = pos.getHeading(AngleUnit.DEGREES);

        TelemetryPacket packet = new TelemetryPacket(false);
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.fieldOverlay().drawImage("/images/biobuzz.png", 0, 0, 144, 144);

        // 1. Draw robot body (18x18 inch square)
        fieldOverlay.setStroke("red");
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.strokeRect(robotX - 9, robotY - 9, 18, 18);

        // 2. Draw heading direction line (12 inches forward)
        double headingRad = Math.toRadians(robotHeading+180);
        double frontX = robotX + Math.cos(headingRad) * 12;
        double frontY = robotY + Math.sin(headingRad) * 12;
        fieldOverlay.strokeLine(robotX, robotY, frontX, frontY);

        // 3. Send numeric values to dashboard variables
        packet.put("Field X (in)", robotX);
        packet.put("Field Y (in)", robotY);
        packet.put("Heading (deg)", robotHeading);

        // Send packet to dashboard
        dashboard.sendTelemetryPacket(packet);
    }
}