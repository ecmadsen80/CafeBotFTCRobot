package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(group = "BIOBUZZ", name = "Meet1")
public class Claude extends LinearOpMode {

    private DcMotorEx leftDrive, leftTurn, rightDrive, rightTurn, intake, flywheelRight, flywheelLeft;
    private Servo gate;
    private I2cDeviceSynch as5600Left, as5600Right;
    private FtcDashboard dashboard;

    private static final double GOBILDA_ENCODER_TICKS = 751.8;
    private static final double TURN_VELOCITY = 2500;
    private static final double DEADZONE = 0.2;
    private static final double TICKS_PER_REV = 28;

    private static final int AS5600_ADDR = 0x36;
    private static final int ANGLE_REGISTER = 0x0E;
    private static final int AS5600_RESOLUTION = 4096;
    private static final int AS5600_RAW_MASK = 0x0FFF;
    private static final double DEGREES_PER_REV = 360.0;

    private static final double LEFT_ZERO_POSITION = 352;
    private static final double RIGHT_ZERO_POSITION = 340;

    private static final double DRIFT_THRESHOLD_DEG = 2.0;
    private static final int SENSOR_READ_INTERVAL = 10;

    private static final double FLYWHEEL_RPM_STEP_SMALL = 25;
    private static final double FLYWHEEL_RPM_STEP_LARGE = 250;
    private static final double FLYWHEEL_RPM_PRESET = 2500;
    private static final double MAX_FLYWHEEL_RPM = 4000;
    private static final double INTAKE_GATE_SCALE = 0.4;
    private static final double SLOW_MODE_LIMIT = 0.5;

    private static final double GATE_DOWN_POSITION = 0.0;
    private static final double GATE_FEED_POSITION = 1.0;

    private static final PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(100, 0.0, 0.0, 10);

    private boolean speedToggle = false;
    private double targetRPM = 0;
    private double leftOffsetDeg = 0, rightOffsetDeg = 0;
    private int cachedRawLeft = 0, cachedRawRight = 0;
    private int loopCount = 0;

    @Override
    public void runOpMode() {
        initHardware();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();

        alignWheels();
        updateSensorCache(true);

        try {
            while (opModeIsActive()) {
                updateSensorCache(false);
                updateDrive();
                updateFlywheels();
                updateIntakeAndGate();
                sendTelemetry();
            }
        } finally {
            stopAllPowered();
        }
    }

    // ---------- Setup ----------

    private void initHardware() {
        leftDrive = hardwareMap.get(DcMotorEx.class, "left_drive");
        leftTurn = hardwareMap.get(DcMotorEx.class, "left_turn");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        rightTurn = hardwareMap.get(DcMotorEx.class, "right_turn");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        gate = hardwareMap.get(Servo.class, "gate");

        as5600Left = hardwareMap.get(Rev2mDistanceSensor.class, "as5600Left").getDeviceClient();
        as5600Left.setI2cAddress(I2cAddr.create7bit(AS5600_ADDR));
        as5600Left.engage();

        as5600Right = hardwareMap.get(Rev2mDistanceSensor.class, "as5600Right").getDeviceClient();
        as5600Right.setI2cAddress(I2cAddr.create7bit(AS5600_ADDR));
        as5600Right.engage();

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetTurnEncoders();

        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL_PIDF);

        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL_PIDF);

        gate.setPosition(GATE_DOWN_POSITION);
        dashboard = FtcDashboard.getInstance();
    }

    /** Rotates both pods to their zero positions before driving starts. */
    private void alignWheels() {
        leftTurn.setPower(1.0);
        rightTurn.setPower(1.0);

        int leftDelta = computeAlignmentDelta(as5600Left, LEFT_ZERO_POSITION);
        leftTurn.setTargetPosition(leftTurn.getCurrentPosition() - leftDelta);
        leftTurn.setVelocity(TURN_VELOCITY);

        int rightDelta = computeAlignmentDelta(as5600Right, RIGHT_ZERO_POSITION);
        rightTurn.setTargetPosition(rightTurn.getCurrentPosition() - rightDelta);
        rightTurn.setVelocity(TURN_VELOCITY);

        while (opModeIsActive() && (leftTurn.isBusy() || rightTurn.isBusy())) {
            telemetry.addLine("Aligning wheels...");
            telemetry.addData("Goal Left", LEFT_ZERO_POSITION);
            telemetry.addData("Actual Left", readAngleDegrees(as5600Left));
            telemetry.addData("Goal Right", RIGHT_ZERO_POSITION);
            telemetry.addData("Actual Right", readAngleDegrees(as5600Right));
            telemetry.update();
        }

        resetTurnEncoders();
        leftOffsetDeg = 0;
        rightOffsetDeg = 0;
    }

    // ---------- Main loop ----------

    /** Reads the sticks, steers both pods, and sets drive power. */
    private void updateDrive() {
        double lx = -gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;

        if (gamepad1.yWasPressed()) speedToggle = !speedToggle;

        double combinedLeftY = ly + turn;
        double combinedRightY = ly - turn;
        double magLeft = Math.hypot(lx, combinedLeftY);
        double magRight = Math.hypot(lx, combinedRightY);

        if (magLeft < DEADZONE && magRight < DEADZONE) {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            correctDriftIfSettled();
            holdPods();
            return;
        }

        double targetAngleLeft = normalizeDegrees(Math.toDegrees(Math.atan2(lx, combinedLeftY)));
        double targetAngleRight = normalizeDegrees(Math.toDegrees(Math.atan2(lx, combinedRightY)));

        double[] leftResult = optimizePod(targetAngleLeft, leftPodDeg());
        double[] rightResult = optimizePod(targetAngleRight, rightPodDeg());

        leftTurn.setTargetPosition(podAngleToTicks(leftResult[0], leftOffsetDeg));
        rightTurn.setTargetPosition(podAngleToTicks(rightResult[0], rightOffsetDeg));
        leftTurn.setVelocity(TURN_VELOCITY);
        rightTurn.setVelocity(TURN_VELOCITY);

        double speedLimit = speedToggle ? SLOW_MODE_LIMIT : 1.0;
        double maxMag = Math.max(1.0, Math.max(magLeft, magRight));

        double leftPower = (magLeft / maxMag) * leftResult[1] * speedLimit;
        double rightPower = (magRight / maxMag) * rightResult[1] * speedLimit;
        leftDrive.setPower(Range.clip(leftPower, -speedLimit, speedLimit));
        rightDrive.setPower(Range.clip(rightPower, -speedLimit, speedLimit));
    }

    /** Adjusts the flywheel RPM from the gamepad and sets both flywheels to match. */
    private void updateFlywheels() {
        if (gamepad1.dpadUpWasPressed()) targetRPM += FLYWHEEL_RPM_STEP_SMALL;
        if (gamepad1.dpadDownWasPressed()) targetRPM -= FLYWHEEL_RPM_STEP_SMALL;
        if (gamepad1.dpadRightWasPressed()) targetRPM += FLYWHEEL_RPM_STEP_LARGE;
        if (gamepad1.dpadLeftWasPressed()) targetRPM -= FLYWHEEL_RPM_STEP_LARGE;
        if (gamepad1.aWasPressed()) targetRPM = 0;
        if (gamepad1.rightBumperWasPressed()) targetRPM = FLYWHEEL_RPM_PRESET;
        targetRPM = Range.clip(targetRPM, 0, MAX_FLYWHEEL_RPM);

        double targetTPS = (targetRPM / 60.0) * TICKS_PER_REV;
        flywheelRight.setVelocity(targetTPS);
        flywheelLeft.setVelocity(targetTPS);
    }

    /** Runs the intake and positions the feed gate. */
    private void updateIntakeAndGate() {
        double intakePower = 0;
        if (gamepad1.right_trigger > 0) intakePower -= 1.0;
        if (gamepad1.left_trigger > 0) intakePower += 1.0;

        boolean feeding = gamepad1.x;
        gate.setPosition(feeding ? GATE_FEED_POSITION : GATE_DOWN_POSITION);
        if (feeding) intakePower *= INTAKE_GATE_SCALE;
        intake.setPower(intakePower);
    }

    /** Sends telemetry to the Driver Station and the FTC Dashboard. */
    private void sendTelemetry() {
        double leftRpm = Math.abs(flywheelLeft.getVelocity()) / TICKS_PER_REV * 60.0;
        double rightRpm = flywheelRight.getVelocity() / TICKS_PER_REV * 60.0;
        double leftDrift = wrapDegrees(sensorPodDeg(cachedRawLeft, LEFT_ZERO_POSITION) - leftPodDeg());
        double rightDrift = wrapDegrees(sensorPodDeg(cachedRawRight, RIGHT_ZERO_POSITION) - rightPodDeg());

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Flywheel Left RPM", leftRpm);
        packet.put("Flywheel Right RPM", rightRpm);
        packet.put("Flywheel Target RPM", targetRPM);
        packet.put("Left Pod Angle (deg)", leftPodDeg());
        packet.put("Right Pod Angle (deg)", rightPodDeg());
        packet.put("Left Pod Drift (deg)", leftDrift);
        packet.put("Right Pod Drift (deg)", rightDrift);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Degrees Left", rawToDegrees(cachedRawLeft));
        telemetry.addData("Degrees Right", rawToDegrees(cachedRawRight));
        telemetry.addData("Left Pod Drift (deg)", leftDrift);
        telemetry.addData("Right Pod Drift (deg)", rightDrift);
        telemetry.addData("Left Turn Power", leftTurn.getPower());
        telemetry.addData("Right Turn Power", rightTurn.getPower());
        telemetry.addData("Left Drive Power", leftDrive.getPower());
        telemetry.addData("Right Drive Power", rightDrive.getPower());
        telemetry.addData("Flywheel Left RPM", leftRpm);
        telemetry.addData("Flywheel Right RPM", rightRpm);
        telemetry.addData("Flywheel Target RPM", targetRPM);
        telemetry.update();
    }

    /** Stops all powered parts when the OpMode ends. */
    private void stopAllPowered() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        intake.setPower(0);
        flywheelRight.setVelocity(0);
        flywheelLeft.setVelocity(0);
    }

    // ---------- Pod hold and drift correction ----------

    /** Holds both pods at their current positions with active control. */
    private void holdPods() {
        leftTurn.setTargetPosition(leftTurn.getCurrentPosition());
        rightTurn.setTargetPosition(rightTurn.getCurrentPosition());
        leftTurn.setVelocity(TURN_VELOCITY);
        rightTurn.setVelocity(TURN_VELOCITY);
    }

    /** When the pods are settled, corrects any offset between motor and absolute sensor angles. */
    private void correctDriftIfSettled() {
        if (leftTurn.isBusy() || rightTurn.isBusy()) return;

        double leftError = wrapDegrees(sensorPodDeg(cachedRawLeft, LEFT_ZERO_POSITION) - leftPodDeg());
        if (abs(leftError) > DRIFT_THRESHOLD_DEG) leftOffsetDeg += leftError;

        double rightError = wrapDegrees(sensorPodDeg(cachedRawRight, RIGHT_ZERO_POSITION) - rightPodDeg());
        if (abs(rightError) > DRIFT_THRESHOLD_DEG) rightOffsetDeg += rightError;
    }

    private double leftPodDeg() {
        return ticksToDegrees(leftTurn.getCurrentPosition()) + leftOffsetDeg;
    }

    private double rightPodDeg() {
        return ticksToDegrees(rightTurn.getCurrentPosition()) + rightOffsetDeg;
    }

    /** Pod angle from the absolute sensor. The -1 sign matches the direction implied by alignment. */
    private double sensorPodDeg(int rawAngle, double zeroPosition) {
        return -wrapDegrees(rawToDegrees(rawAngle) - zeroPosition);
    }

    private int podAngleToTicks(double podDeg, double offsetDeg) {
        return (int) Math.round(degreesToTicks(podDeg - offsetDeg));
    }

    /** Refreshes cached AS5600 readings every SENSOR_READ_INTERVAL loops, or immediately if forced. */
    private void updateSensorCache(boolean force) {
        loopCount++;
        if (!force && loopCount % SENSOR_READ_INTERVAL != 0) return;
        cachedRawLeft = readRawAngle(as5600Left);
        cachedRawRight = readRawAngle(as5600Right);
    }

    // ---------- Helpers ----------

    private void resetTurnEncoders() {
        leftTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftTurn.setTargetPosition(0);
        rightTurn.setTargetPosition(0);
        leftTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private int computeAlignmentDelta(I2cDeviceSynch as5600, double zeroPosition) {
        double angleError = wrapDegrees(zeroPosition - readAngleDegrees(as5600));
        return (int) Math.round(angleError / DEGREES_PER_REV * GOBILDA_ENCODER_TICKS);
    }

    /** Picks the shorter rotation; if over 90 degrees, turns to the opposite angle and drives in reverse. */
    private double[] optimizePod(double targetAngle, double currentDeg) {
        double driveMult = 1.0;
        if (abs(closestAngle(targetAngle, currentDeg)) > 90) {
            driveMult = -1.0;
            targetAngle = (targetAngle + 180) % DEGREES_PER_REV;
        }
        double moveDeg = closestAngle(targetAngle, currentDeg) + currentDeg;
        return new double[]{moveDeg, driveMult};
    }

    /** Wraps an angle into [-180, 180). */
    private double wrapDegrees(double angle) {
        double a = (angle + 180.0) % DEGREES_PER_REV;
        if (a < 0) a += DEGREES_PER_REV;
        return a - 180.0;
    }

    /** Normalizes an angle into [0, 360). */
    private double normalizeDegrees(double angle) {
        double a = angle % DEGREES_PER_REV;
        return a < 0 ? a + DEGREES_PER_REV : a;
    }

    private double ticksToDegrees(int ticks) {
        return (ticks / GOBILDA_ENCODER_TICKS) * DEGREES_PER_REV;
    }

    private double degreesToTicks(double degrees) {
        return (degrees / DEGREES_PER_REV) * GOBILDA_ENCODER_TICKS;
    }

    /** Reads the 12-bit raw angle from an AS5600 (high byte first). */
    private int readRawAngle(I2cDeviceSynch as5600) {
        byte[] b = as5600.read(ANGLE_REGISTER, 2);
        return (((b[0] & 0xFF) << 8) | (b[1] & 0xFF)) & AS5600_RAW_MASK;
    }

    private double rawToDegrees(int rawAngle) {
        return rawAngle * DEGREES_PER_REV / AS5600_RESOLUTION;
    }

    private double readAngleDegrees(I2cDeviceSynch as5600) {
        return rawToDegrees(readRawAngle(as5600));
    }

    /** Shortest signed difference from current to target, in [-180, 180). */
    private double closestAngle(double target, double current) {
        return wrapDegrees(target - current);
    }
}
