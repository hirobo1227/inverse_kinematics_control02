from robot_arm.gamepad import PS5Controller, apply_deadzone
from robot_arm.control import CanTransport, MotorController
from robot_arm.kinematics import forwardKinematics, inverseKinmatics
from robot_arm.motor_config import modelsById, limitsById, MOTOR_1_CAN_ID, MOTOR_2_CAN_ID, MOTOR_3_CAN_ID, MOTOR_4_CAN_ID
import time
import math


MOTOR_1_CAN_ID = 1
MOTOR_2_CAN_ID = 2
MOTOR_3_CAN_ID = 3
MOTOR_4_CAN_ID = 4

RIGHT_AXIS_X = 2
RIGHT_AXIS_Y = 5
LEFT_AXIS_X = 0
LEFT_AXIS_Y = 1
LEFT_PUSH_AXIS = 3
RIGHT_PUSH_AXIS = 4


MAX_SPEED_X_M = 0.3
MAX_SPEED_Y_M = 0.3
MAX_SPEED_Z_M = 0.3
MAX_SPEED_EE_RAD_S = 35

def main():
    try:
        with PS5Controller() as controller:
            print(controller.name)

            transport = CanTransport(channel="can0")
            motorController = None

            try:
                
                motorController = MotorController(
                    transport,
                    modelsById,
                    limitsById,
                )

                firstTime = True

                motorIds = [
                    MOTOR_1_CAN_ID,
                    MOTOR_2_CAN_ID,
                    MOTOR_3_CAN_ID,
                    MOTOR_4_CAN_ID,
                ]

                previousTime = time.monotonic()

                while True:

                    try:
                        m1State = transport.read_motor(MOTOR_1_CAN_ID, timeout_s=0.1)
                        m2State = transport.read_motor(MOTOR_2_CAN_ID, timeout_s=0.1)
                        m3State = transport.read_motor(MOTOR_3_CAN_ID, timeout_s=0.1)
                        m4State = transport.read_motor(MOTOR_4_CAN_ID, timeout_s=0.1)
                    except TimeoutError as error:
                        print("受信タイムアウト: {error}")

                        break

                    for state in (m1State, m2State, m3State, m4State):
                        if state.fault:
                            raise RuntimeError(
                                f'Motor {state.can_id}: fault code {state.fault}'
                            )

                    if(firstTime):
                        firstTime = False
                        targetPosition_x, targetPosition_y, targetPosition_z = forwardKinematics(
                            m1State.position_rad,
                            m2State.position_rad,
                            m3State.position_rad,
                        )
                        targetEndEffectorAngle_rad = m4State.position_rad


                    pad = controller.read_controller()

                    if not pad.connected:
                        print("コントローラが切断されました")
                        break

                    leftStick_x = apply_deadzone(pad.axes[LEFT_AXIS_X], 0.4)
                    leftStick_y = apply_deadzone(pad.axes[LEFT_AXIS_Y], 0.4)
                    rightStick_y = apply_deadzone(pad.axes[RIGHT_AXIS_Y], 0.4)

                    leftPushAxis = apply_deadzone((pad.axes[LEFT_PUSH_AXIS] + 1.0) / 2.0, 0.1)
                    rightPushAxis = apply_deadzone((pad.axes[RIGHT_PUSH_AXIS] + 1.0) / 2.0, 0.1)

                    targetSpeed_x = leftStick_x * MAX_SPEED_X_M
                    targetSpeed_y = -leftStick_y * MAX_SPEED_Y_M
                    targetSpeed_z = -rightStick_y * MAX_SPEED_Z_M

                    if(rightPushAxis):
                        targetSpeed_ee = rightPushAxis * -MAX_SPEED_EE_RAD_S
                    elif(leftPushAxis):
                        targetSpeed_ee = leftPushAxis * MAX_SPEED_EE_RAD_S
                    else:
                        targetSpeed_ee = 0.0

                    
                    now = time.monotonic()
                    dt = now - previousTime
                    previousTime = now

                    #print(targetPosition_x, targetPosition_y, targetPosition_z)

                    candidatePosition = (
                        targetPosition_x + targetSpeed_x * dt,
                        targetPosition_y + targetSpeed_y * dt,
                        targetPosition_z + targetSpeed_z * dt,
                    )

                    angles = inverseKinmatics(*candidatePosition)
                    

                    if angles is not None:
                        targetPosition_x, targetPosition_y, targetPosition_z = candidatePosition
                        targetAxis1Angle_rad, targetAxis2Angle_rad, targetAxis3Angle_rad = angles

                        motorController.command_motor(
                            can_id=MOTOR_1_CAN_ID,
                            position_rad=targetAxis1Angle_rad,
                            velocity_rad_s=0.0,
                            kp=20.0,
                            kd=3.0,
                            torque_nm=0.0,
                        )
                        motorController.command_motor(
                            can_id=MOTOR_2_CAN_ID,
                            position_rad=targetAxis2Angle_rad,
                            velocity_rad_s=0.0,
                            kp=10.0,
                            kd=0.8,
                            torque_nm=0.0,
                        )
                        motorController.command_motor(
                            can_id=MOTOR_3_CAN_ID,
                            position_rad=targetAxis3Angle_rad,
                            velocity_rad_s=0.0,
                            kp=10.0,
                            kd=0.8,
                            torque_nm=0.0,
                        )
                        motorController.command_motor(
                            can_id=MOTOR_4_CAN_ID,
                            position_rad=targetAxis1Angle_rad,
                            #position_rad=0.0,
                            #velocity_rad_s=targetSpeed_ee,
                            velocity_rad_s=0.0,
                            kp=3.0,
                            kd=0.1,
                            torque_nm=0.0,
                        )

   
                    
            finally:
                if(motorController != None):
                    motorController.disable_all()
                
                transport.close()

                
                
                


    except KeyboardInterrupt:
        print("終了しました")

    except Exception as error:
        print(f"エラーが発生しました: {error}")



if __name__ == "__main__":
    main()