"""モーターの機種情報・CAN ID・指令値の制限。

位置: rad、速度: rad/s、トルク: Nm。
"""
import math

# CAN ID
MOTOR_1_CAN_ID = 1
MOTOR_2_CAN_ID = 2
MOTOR_3_CAN_ID = 3
MOTOR_4_CAN_ID = 4


# 通信データの変換範囲。運用上の制限とは別。
AK40_10_MODEL = {
    "position_rad": (-12.56, 12.56),
    "velocity_rad_s": (-45.0, 45.0),
    "torque_nm": (-5.0, 5.0),
    "kp": (0.0, 500.0),
    "kd": (0.0, 5.0),
}

AK45_36_MODEL = {
    "position_rad": (-12.56, 12.56),
    "velocity_rad_s": (-6.0, 6.0),
    "torque_nm": (-34.0, 34.0),
    "kp": (0.0, 500.0),
    "kd": (0.0, 5.0),
}


# 各軸で使用するモデル
modelsById = {
    MOTOR_1_CAN_ID: AK40_10_MODEL,
    MOTOR_2_CAN_ID: AK45_36_MODEL,
    MOTOR_3_CAN_ID: AK45_36_MODEL,
    MOTOR_4_CAN_ID: AK40_10_MODEL,
}


# 実機に合わせて設定する指令値の制限。
# NoneのままではMotorControllerを初期化できません。
# set_motor_torque()では位置・速度の指令値が0になるため、各範囲に0が必要です。
# 実測位置・速度の監視とは別です。
limitsById = {
    MOTOR_1_CAN_ID: {
        "position_rad": (math.radians(-100), math.radians(100)),
        "velocity_rad_s": (math.radians(-2200), math.radians(2200)),
        "torque_nm": (-4.1, 4.1),
    },
    MOTOR_2_CAN_ID: {
        "position_rad": (0, math.radians(105)),
        "velocity_rad_s": (math.radians(-240), math.radians(240)),
        "torque_nm": (-8, 8),
    },
    MOTOR_3_CAN_ID: {
        "position_rad": (math.radians(-20), math.radians(100)),
        "velocity_rad_s": (math.radians(-240), math.radians(240)),
        "torque_nm": (-8, 8),
    },
    MOTOR_4_CAN_ID: {
        "position_rad": (math.radians(-100), math.radians(100)),
        "velocity_rad_s": (math.radians(-2200), math.radians(2200)),
        "torque_nm": (-4.1, 4.1),
    },
}