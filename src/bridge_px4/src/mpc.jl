#!/usr/bin/env julia

using IterativeLQR
using mpc_data
using MAT
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose2D, PointStamped, QuaternionStamped, PoseStamped
@rosimport std_msgs.msg: Float32MultiArray
rostypegen()
using .geometry_msgs.msg
using .std_msgs.msg
using Plots


function callback(msg::Float32MultiArray, current_z)
    # print("in callback\n")
    current_z .= convert(Vector{Float32}, msg.data)
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

function plot_current_motion_plan(x_sol, u_sol, xref, num_steps, i)
    h = plot([x[7] for x in xref[i .+ (1:num_steps)]], [x[8] for x in xref[i .+ (1:num_steps)]], linewidth=2, label="tip_desired", aspect_ratio=:equal)
    plot!([x[7] for x in x_sol], [x[8] for x in x_sol], linewidth=2, label="tip_plan", aspect_ratio=:equal)
    plot!([x[1] for x in x_sol], [x[2] for x in x_sol], linewidth=2, label="drone_plan", aspect_ratio=:equal)
    plot!([u[1] for u in u_sol], [u[2] for u in u_sol], linewidth=2, label="control_plan", aspect_ratio=:equal)
    xlims!(-1.5, 1.5)
    ylims!(-1.0, 1.0)
    display(h)
end

function loop(pos_pub_obj, att_pub_obj, mpc_data, tip_measurement, drone_measurement, current_koopman_state; num_steps = 200, override=true)
    i = 0
    x_sol, u_sol = mpc_data.x_sol, mpc_data.u_sol
    U = zeros(3, num_steps)
    Z = zeros(45, num_steps)
    Z1 = zeros(45, num_steps)
    X_drone = zeros(3, num_steps)
    X_tip = zeros(3, num_steps)
    # reset_state!(x_sol[2])
    # reset_measurement!(tip_measurement, drone_measurement)

    loop_rate = Rate(20.0)
    for i in 1:num_steps
        t_now = RobotOS.now()
        
        if ! override
            # update_state!(x_sol[2], tip_measurement, drone_measurement)
            x_sol[2] .= current_koopman_state
        end

        # print("State: ", round.(x_sol[2][1:3]; digits=4), "\n")
        # update_problem!(mpc_data.solver, mpc_data.objectives[i], mpc_data.model, x_sol, [u_sol[2:end]..., u_sol[end]])
        update_problem!(mpc_data.solver, mpc_data.objectives[i+5], mpc_data.model, x_sol, mpc_data.uref[(i-1) .+ (1:length(mpc_data.model))])
        # update_problem!(mpc_data.solver, mpc_data.objectives[i], mpc_data.model, x_sol, mpc_data.uref[1:24])
        
        x_sol, u_sol = get_trajectory(mpc_data.solver)

        publish_pose_cmd(pos_pub_obj, att_pub_obj, RobotOS.now(), i, u_sol[1])

        print("Iteration: ", i, " Time: ", (RobotOS.now() - t_now).nsecs/10^9, "\n")
        
        # X_drone[1, i] = drone_measurement.msg.pose.position.x
        # X_drone[2, i] = drone_measurement.msg.pose.position.y
        # X_drone[3, i] = drone_measurement.msg.pose.position.z
        # X_tip[1, i] = tip_measurement.msg.pose.position.x
        # X_tip[2, i] = tip_measurement.msg.pose.position.y
        # X_tip[3, i] = tip_measurement.msg.pose.position.z
        Z1[:, i] = x_sol[1]
        Z[:, i] = x_sol[2]
        U[:, i] = u_sol[1]

        plot_current_motion_plan(x_sol, u_sol, mpc_data.xref, length(x_sol), i)

        # readline()

        rossleep(loop_rate)
    end

    return Z, U, X_drone, X_tip, Z1
end

init_node("mpc_node")

# subscribers
# drone_measurement = PoseMeasurement(PoseStamped())
# tip_measurement = PoseMeasurement(PoseStamped())

# Subscriber{PoseStamped}("/vrpn_client_node/tip/pose", callback, (tip_measurement,), queue_size=1)
# Subscriber{PoseStamped}("/drone5/mavros/vision_pose/pose", callback, (drone_measurement,), queue_size=1)

current_koopman_state = zeros(45)
Subscriber{Float32MultiArray}("/z_flying_vine", callback, (current_koopman_state,), queue_size=1)

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
print("Problem data initialized\n")
# Z, U, X, Z1 = loop(pos_pub, att_pub, problem_data, tip_measurement, drone_measurement, 
#             num_steps=10, override=false)

# display(plot(U[1,:], U[2,:], linewidth=2, label="control", aspect_ratio=:equal))
# display(plot(Z[1,:], Z[2,:], linewidth=2, label="position", aspect_ratio=:equal))
# display(plot(X[1,:], X[2,:], linewidth=2, label="drone", aspect_ratio=:equal))

function plot_results(show_control=false, show_reference=false)
    h = plot([0],[0])
    plot!(Z[1,:], Z[2,:], linewidth=2, label="Z drone", aspect_ratio=:equal)
    plot!(Z[7,:], Z[8,:], linewidth=2, label="Z tip", aspect_ratio=:equal)
    plot!(Z1[1,:], Z1[2,:], linewidth=2, label="Z1 drone", aspect_ratio=:equal)
    plot!(Z1[7,:], Z1[8,:], linewidth=2, label="Z1 tip", aspect_ratio=:equal)
    plot!(X_drone[1,:], X[2,:], linewidth=2, label="drone", aspect_ratio=:equal, linestyle=:dash)
    plot!(X_tip[1,:], X_tip[2,:], linewidth=2, label="tip", aspect_ratio=:equal, linestyle=:dash)
    if show_control
        plot!(U[1,:], U[2,:], linewidth=2, label="control", aspect_ratio=:equal)
    end
    if show_reference
        num_steps = length(Z[1,:])
        plot!([x[7] for x in xref[1:num_steps]], [x[8] for x in xref[1:num_steps]], linewidth=2, label="tip reference", aspect_ratio=:equal)
    end
    display(h)
end

# Z, U, X_drone, X_tip, Z1 = loop(pos_pub, att_pub, problem_data, 1, 1, current_koopman_state, num_steps=14, override=false);


# # Load real control
# mat_file = matopen("/home/oem/flyingSysID/fig8_sim_real_history_size_5.mat")
# U_real = read(mat_file, "U")
# close(mat_file)

# plot(U[1,:], U[2,:], linewidth=2, label="control", aspect_ratio=:equal)
# # plot!(Z1[1,:], Z1[2,:], linewidth=2, label="Z1 drone", aspect_ratio=:equal)
# plot!(Z1[7,:], Z1[8,:], linewidth=2, label="Z1 tip", aspect_ratio=:equal)
# num_steps = length(Z[1,:])
# plot!([x[7] for x in xref[1:num_steps]], [x[8] for x in xref[1:num_steps]], linewidth=2, label="tip reference", aspect_ratio=:equal)
# # plot!(U_real[1,:], U_real[2,:], linewidth=2, label="real control", aspect_ratio=:equal)