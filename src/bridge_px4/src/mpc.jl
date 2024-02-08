#!/usr/bin/env julia

using IterativeLQR
using mpc_data
using MAT
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose2D, PointStamped, QuaternionStamped, PoseStamped
rostypegen()
using .geometry_msgs.msg


struct PoseMeasurement
    msg
end

function callback(msg::PoseStamped, measurement)
    measurement.msg = msg
end

function publish_pose_cmd(pos_pub_obj, att_pub_obj, t_now, i, u)
    # Variables to publish
    pos_msg = PointStamped()
    att_msg = QuaternionStamped()

    # Position
    pos_msg.header.stamp = t_now
    pos_msg.header.seq = i
    pos_msg.header.frame_id = "map"

    pos_msg.point.x = clamp(u[1], -2.0, 2.0)
    pos_msg.point.y = clamp(u[2], -2.0, 2.0)
    pos_msg.point.z = clamp(u[3], -2.0, 2.0)

    # Attitude
    att_msg.header.stamp = t_now
    att_msg.header.seq = i
    att_msg.header.frame_id = "map"
    
    att_msg.quaternion.w = 1.0
    att_msg.quaternion.x = 0.0
    att_msg.quaternion.y = 0.0
    att_msg.quaternion.z = 0.0

    publish(pos_pub_obj, pos_msg)
    publish(att_pub_obj, att_msg)
    
end

function loop(pos_pub_obj, att_pub_obj, mpc_data, tip_measurement, drone_measurement)
    i = 0
    x_sol, u_sol = mpc_data.x_sol, mpc_data.u_sol

    loop_rate = Rate(20.0)
    while ! is_shutdown()

        if i == 200
            return
        end
        
        i += 1
        t_now = RobotOS.now()
        
        # use the latest measurements
        x_sol[2][1:3] = [drone_measurement.msg.pose.position.x, drone_measurement.msg.pose.position.y, drone_measurement.msg.pose.position.z]
        x_sol[2][4:6] = [drone_measurement.msg.pose.orientation.x, drone_measurement.msg.pose.orientation.y, drone_measurement.msg.pose.orientation.z]
        x_sol[2][7:9] = [tip_measurement.msg.pose.position.x, tip_measurement.msg.pose.position.y, tip_measurement.msg.pose.position.z]
        
        update_problem!(mpc_data.solver, mpc_data.objectives[i], mpc_data.model, x_sol, [u_sol[2:end]..., u_sol[end]])
        x_sol, u_sol = get_trajectory(mpc_data.solver)

        publish_pose_cmd(pos_pub_obj, att_pub_obj, RobotOS.now(), i, u_sol[1])

        print("Iteration: ", i, " Time: ", (RobotOS.now() - t_now).nsecs/10^9, "\n")

        rossleep(loop_rate)
    end
end

init_node("mpc_node")

# subscribers
drone_measurement = PoseMeasurement(PoseStamped())
tip_measurement = PoseMeasurement(PoseStamped())

Subscriber{PoseStamped}("/vrpn_client_node/tip/pose", callback, (tip_measurement,))
Subscriber{PoseStamped}("/drone5/mavros/vision_pose/pose", callback, (drone_measurement,))

# publishers
pos_pub = Publisher{PointStamped}("/gcs/setpoint/position",queue_size=1)
att_pub = Publisher{QuaternionStamped}("/gcs/setpoint/attitude",queue_size=1)

# Load dynamics
mat_file = matopen("/home/oem/flyingSysID/deflated_sysID_nice-resample_dt0-05_history-size-5.mat")
A_full = read(mat_file, "A_full")
B_full = read(mat_file, "B_full")
close(mat_file)

# MPC loop
T = 25
T2 = 400
xref, uref = gen_Z_from_tip(T2, 5, 200) #T2, history_size, period
problem_data = MPC(T, T2, A_full, B_full, xref, uref);
loop(pos_pub, att_pub, problem_data, tip_measurement, drone_measurement)
