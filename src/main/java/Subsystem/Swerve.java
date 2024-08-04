package Subsystem;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotMap;

import static edu.wpi.first.units.Units.Meters;

public class Swerve extends SubsystemBase {
    private WPI_Pigeon2 pigeon;
    private SwerveDriveOdometry odometry;
    private SwerveModule[] swerveModules;
    private SwerveDriveKinematics kinematics;
    SysIdRoutine sysIdRoutine;
    public Swerve(SwerveModule[] swerveModules){
        sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(this::volatageDrive,this::sysidLog,this));
        this.swerveModules = swerveModules;
        double distance = 0.37;
        this.kinematics = new SwerveDriveKinematics(
                new Translation2d(distance, distance),
                new Translation2d(distance, -distance),
                new Translation2d(-distance, distance),
                new Translation2d(-distance,-distance)
        );
        pigeon = new WPI_Pigeon2(RobotMap.PIGEON);
        odometry = new SwerveDriveOdometry(kinematics,pigeon.getRotation2d(),getModulesPosition());

    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public SwerveModulePosition[] getModulesPosition(){
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        for(int i =0; i<4; i++){
            swerveModulePositions[i] = swerveModules[i].getModulePosition();
        }

        return swerveModulePositions;
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        for(int i =0; i <4; i++){
            swerveModuleStates[i] = swerveModules[i].getModuleStates();
        }

        return swerveModuleStates;
    }

    public ChassisSpeeds getSpeeds(){
        return kinematics.toChassisSpeeds(new SwerveDriveKinematics.SwerveDriveWheelStates(getModuleStates()));
    }

    public void sysidLog(SysIdRoutineLog log) {
        SwerveModule frontLeft = swerveModules[0];
        SwerveModule frontRight = swerveModules[1];

        log.motor("drive-left")
                .voltage(frontLeft.getOutputVoltage())
                .linearPosition(Meters.of(frontLeft.getPositionMeters()))
                .linearVelocity(frontLeft.getLinearVelocity());

        log.motor("drive-right")
                .voltage(frontRight.getOutputVoltage())
                .linearPosition(Meters.of(frontRight.getPositionMeters()))
                .linearVelocity(frontRight.getLinearVelocity());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public void drive(double speedX, double speedY, double rotation) {
        SmartDashboard.putNumber("SwerveCommandX", speedX);
        SmartDashboard.putNumber("SwerveCommandY", speedY);
        SmartDashboard.putNumber("SwerveCommandRot", rotation);

        SwerveModuleState[] swerveModuleStates;
        swerveModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(speedY,speedX,rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,10);
        setDesiredState(swerveModuleStates);
    }

    public void drive(ChassisSpeeds speeds){
        SwerveModuleState[] swerveModuleStates;
        swerveModuleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,4.4196);
        setDesiredState(swerveModuleStates);
    }

    public void volatageDrive(Measure<Voltage> voltage){
        double volts = voltage.in(Units.Volts);
        double output = volts / RobotController.getBatteryVoltage();

        move(output, 0);
    }

    public void setDesiredState(SwerveModuleState[] swerveModuleStates){
        for(int i=0; i<4; i++){
            swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    public void move(double drive, double rotation) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].move(drive, rotation);
        }
    }

    public void setDriveVelocity(double velocityMps) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setDriveVelocity(velocityMps);
        }
    }

    public void setSteerPosition(double positionDegrees) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setSteerPosition(positionDegrees);
        }
    }

    public void stop(){
        for(int i =0; i <4; i++){
            swerveModules[i].stop();
        }
    }

    @Override
    public void periodic() {
        odometry.update(pigeon.getRotation2d(),getModulesPosition());

        for(int i =0; i <4; i++){
            swerveModules[i].periodic();
        }
    }
}
