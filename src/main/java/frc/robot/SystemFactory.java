package frc.robot;

import Subsystem.Swerve;
import Subsystem.SwerveModule;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class SystemFactory {
    public SystemFactory(){

    }

    public static Swerve createSwerve(){

        SwerveModule[] swerveModules = new SwerveModule[] {
                createInvertedModule(51, 52, 5,RobotMap.SWEARVE_ABSOLUTE_FL_ZERO_ANGLE, "LF"),//LF
                createInvertedModule(41, 42, 4,RobotMap.SWEARVE_ABSOLUTE_FR_ZERO_ANGLE, "RF"),//RF
                createInvertedModule(61, 62, 6,RobotMap.SWEARVE_ABSOLUTE_RL_ZERO_ANGLE, "LB"),//LB
                createInvertedModule(31, 32, 3,RobotMap.SWEARVE_ABSOLUTE_RR_ZERO_ANGLE, "RB"),//RB

        };
        return new Swerve(swerveModules);
    }


    public static SwerveModule createInvertedModule(int drive, int steer, int encoder, double zeroAngle, String identifier) {
        CANSparkMax spark = new CANSparkMax(drive, CANSparkLowLevel.MotorType.kBrushless);
        spark.restoreFactoryDefaults();
        spark.setInverted(true);

        CANSparkMax steerMotor = new CANSparkMax(steer, CANSparkLowLevel.MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();

        return new SwerveModule(spark,
                steerMotor,
                new CANCoder(encoder),
                zeroAngle,
                identifier);
    }
}
