package frc.robot;

import edu.wpi.first.math.util.Units;

public class RobotMap {
    public static final int driveRightConnection =1;
    public static final int driveLeftConnection =0;
    public static final double DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO = 8.4;
    public static final double DRIVE_WHEEL_RADIUS_M = Units.inchesToMeters(2.5);
    public static final double DRIVE_WHEEL_DIAMETER_M = 2 * Math.PI * DRIVE_WHEEL_RADIUS_M;
    public static final int DRIVE_SIDE_MOTOR_COUNT = 4;
    public static final double WEIGHT_KG = 40;
    public static final double DRIVE_TRACK_WIDTH_M = 4;
    public static final double ROBOT_WIDTH_M = DRIVE_TRACK_WIDTH_M;
    public static final double ROBOT_LENGTH_M = 5;
    public static final double DRIVE_MOMENT_OF_INERTIA = (Math.pow(ROBOT_LENGTH_M, 3) * ROBOT_WIDTH_M) / 12;
    public static final double MOVE_SPEED = 5;
    public static final int PIGEON =9;
    public static final double CHASSIS_RADIUS = 0.63;


    public static final double SWEARVE_ABSOLUTE_FL_ZERO_ANGLE = 91.2;
    public static final double SWEARVE_ABSOLUTE_FR_ZERO_ANGLE = 297.59;
    public static final double SWEARVE_ABSOLUTE_RL_ZERO_ANGLE = 35.1;
    public static final double SWEARVE_ABSOLUTE_RR_ZERO_ANGLE = 190;





}
