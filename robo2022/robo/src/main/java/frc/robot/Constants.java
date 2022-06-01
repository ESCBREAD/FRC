package frc.robot;

public class Constants {

    //1 设置-Math.toRadians(0.0);
    //2 上电，地盘设定0位置
    //3 对齐轮子，朝北，齿朝左
    //4 上电，shuffleboard获取当前轮子位置
    //5 修改-Math.toRadians(87.01054687500002); 到当前位置
    //6 从新部署，运行
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.2794;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.2794;

    public static final int DRIVETRAIN_PIGEON_ID = 0;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(87.01054687500002);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(258.6609375);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(130.6921875);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(326.86171875);



    // public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3;
    // public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4;
    // public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10;
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

    // public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1;
    // public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
    // public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9;
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

    // public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    // public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    // public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

    // public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    // public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    // public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
}
