#!/usr/bin/env julia

using IterativeLQR
using mpc_data
using MAT
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose2D, PointStamped, QuaternionStamped, PoseStamped
rostypegen()
using .geometry_msgs.msg
using Plots


struct PoseMeasurement
    msg
end

function callback(msg::PoseStamped, measurement)
    print(msg.pose.position)
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

function update_state!(x, tip_measurement, drone_measurement)
    # shift the old configurations
    x[10:end] = x[1:end-9]
    
    # fill in the current configuration
    x[1:3] = [drone_measurement.msg.pose.position.x, drone_measurement.msg.pose.position.y, drone_measurement.msg.pose.position.z]
    x[4:6] = [drone_measurement.msg.pose.orientation.x, drone_measurement.msg.pose.orientation.y, drone_measurement.msg.pose.orientation.z]
    x[7:9] = [tip_measurement.msg.pose.position.x, tip_measurement.msg.pose.position.y, tip_measurement.msg.pose.position.z]
    return
end

function reset_state!(x)
    x[1:9:end] .= 0.011
    x[2:9:end] .= 0.017
    x[3:9:end] .= 1.477

    x[4:9:end] .= 0.130
    x[5:9:end] .= -0.121
    x[6:9:end] .= -0.007

    x[7:9:end] .= 0.020
    x[8:9:end] .= -0.026
    x[9:9:end] .= 0.501
end

function reset_measurement!(tip_measurement, drone_measurement)
    drone_measurement.msg.pose.position.x = 0.011
    drone_measurement.msg.pose.position.y = 0.017
    drone_measurement.msg.pose.position.z = 1.477
    
    drone_measurement.msg.pose.orientation.x = 0.130
    drone_measurement.msg.pose.orientation.y = -0.121
    drone_measurement.msg.pose.orientation.z = -0.007
    
    tip_measurement.msg.pose.position.x = 0.020
    tip_measurement.msg.pose.position.y = -0.026
    tip_measurement.msg.pose.position.z = 0.501
end



function loop(pos_pub_obj, att_pub_obj, mpc_data, tip_measurement, drone_measurement; num_steps = 200, override=true)
    i = 0
    x_sol, u_sol = mpc_data.x_sol, mpc_data.u_sol
    U = zeros(3, num_steps)
    Z = zeros(45, num_steps)
    reset_state!(x_sol[2])
    reset_measurement!(tip_measurement, drone_measurement)

    loop_rate = Rate(20.0)
    for i in 1:num_steps
        t_now = RobotOS.now()
        
        if ! override
            update_state!(x_sol[2], tip_measurement, drone_measurement)
        end

        print("State: ", x_sol[2][1:9], "\n")
        update_problem!(mpc_data.solver, mpc_data.objectives[i], mpc_data.model, x_sol, [u_sol[2:end]..., u_sol[end]])
        
        x_sol, u_sol = get_trajectory(mpc_data.solver)

        publish_pose_cmd(pos_pub_obj, att_pub_obj, RobotOS.now(), i, u_sol[1])

        print("Iteration: ", i, " Time: ", (RobotOS.now() - t_now).nsecs/10^9, "\n")
        
        Z[:, i] = x_sol[2]
        U[:, i] = u_sol[1]

        rossleep(loop_rate)
    end

    return Z, U
end

init_node("mpc_node")

# subscribers
drone_measurement = PoseMeasurement(PoseStamped())
tip_measurement = PoseMeasurement(PoseStamped())

Subscriber{PoseStamped}("/vrpn_client_node/tip/pose", callback, (tip_measurement,))
Subscriber{PoseStamped}("/drone5/mavros/vision_pose/pose", callback, (drone_measurement,))

# publishers
pos_pub = Publisher{PointStamped}("/gcs/setpoint/position2",queue_size=1)
att_pub = Publisher{QuaternionStamped}("/gcs/setpoint/attitude2",queue_size=1)

# Load dynamics
mat_file = matopen("/home/oem/flyingSysID/deflated_sysID_nice-resample_dt0-05_history-size-5.mat")
A_full = read(mat_file, "A_full")
B_full = read(mat_file, "B_full")
close(mat_file)

# MPC loop
T = 25
T2 = 400
xref, uref = gen_Z_from_tip(T2, 5, 200) #T2, history_size, period
# plot_reference(xref, uref)
problem_data = MPC(T, T2, A_full, B_full, xref, uref);
Z, U = loop(pos_pub, att_pub, problem_data, tip_measurement, drone_measurement, 
            num_steps=10, override=false)

display(plot(U[1,:], U[2,:], linewidth=2, label="control", aspect_ratio=:equal))
display(plot(Z[1,:], Z[2,:], linewidth=2, label="position", aspect_ratio=:equal))