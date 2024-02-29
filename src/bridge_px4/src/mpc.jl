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
using LinearAlgebra
using CSV, DataFrames


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
    pos_msg.point.z = clamp(u[3], 1.0, 2.0)

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

function plot_current_motion_plan(x_sol, u_sol, xref, num_steps, i)
    h = plot([x[7] for x in xref[i .+ (1:num_steps)]], [x[8] for x in xref[i .+ (1:num_steps)]], linewidth=2, label="tip_desired", aspect_ratio=:equal)
    plot!([x[7] for x in x_sol], [x[8] for x in x_sol], linewidth=2, label="tip_plan", aspect_ratio=:equal)
    plot!([x[1] for x in x_sol], [x[2] for x in x_sol], linewidth=2, label="drone_plan", aspect_ratio=:equal)
    plot!([u[1] for u in u_sol], [u[2] for u in u_sol], linewidth=2, label="control_plan", aspect_ratio=:equal)
    xlims!(-1.5, 1.5)
    ylims!(-1.0, 1.0)
    display(h)
end

function safety_check(current_koopman_state)
    # return true # disable safety check

    # if tip measurement has changed too much
    current_tip_position = current_koopman_state[7:9]
    last_tip_position = current_koopman_state[9 .+ (7:9)]
    if norm(current_tip_position - last_tip_position) > 0.5
        print("SAFETY CHECK FAILED: tip measurement changed too much\n")
        print("Current: ", current_tip_position, " Last: ", last_tip_position, "\n")
        return false
    end

    # if tip measurement is too far from drone
    current_drone_position = current_koopman_state[1:3]
    if norm(current_drone_position - current_tip_position) > 2.5
        print("SAFETY CHECK FAILED: tip too far from drone\n")
        print("Tip: ", current_tip_position, " Drone: ", current_drone_position, "\n")
        return false
    end

    return true

end

function loop(pos_pub_obj, att_pub_obj, pos_pub_obj2, att_pub_obj2, mpc_data, current_koopman_state; alpha = .2, num_steps = 200, override=true, plot_motion_plan=false, initial_run=false)
    i = 0
    
    x_sol, u_sol = mpc_data.x_sol, mpc_data.u_sol
    
    U = zeros(3, num_steps)
    Uf = zeros(3, num_steps)
    Z1 = zeros(45, num_steps)
    Z2 = zeros(45, num_steps)
    
    u_filtered = [0.0, 0.0, 1.5]    

    # open CSV for commands to send to real system
    df = CSV.read("/home/oem/StanfordMSL/TrajBridge/src/bridge_px4/trajectories/EE_fig8_10s_motionplan.csv", DataFrame)

    # reset and wait for the first koopman_state
    current_koopman_state .= 0.0
    rossleep(3.0)
    if (current_koopman_state[3] == 0.0) && !initial_run
        print("Koopman state not received\n")
        return 0,0,0,0,0,0
    end
    
    loop_rate = Rate(20.0)
    for i in 1:num_steps
        t_now = RobotOS.now()
        
        if ! override
            x_sol[2] .= current_koopman_state
        end

        if ! safety_check(current_koopman_state)
            break
        end

        # print("State: ", round.(x_sol[2][1:3]; digits=4), "\n")
        # update_problem!(mpc_data.solver, mpc_data.objectives[i], mpc_data.model, x_sol, [u_sol[2:end]..., u_sol[end]])
        update_problem!(mpc_data.solver, mpc_data.objectives[i+5], mpc_data.model, x_sol, mpc_data.uref[(i-1) .+ (1:length(mpc_data.model))])
        
        x_sol, u_sol = get_trajectory(mpc_data.solver)

        u_filtered = alpha * u_sol[1] + (1 - alpha) * u_filtered

        if override
            publish_pose_cmd(pos_pub_obj, att_pub_obj, RobotOS.now(), i, df[1:3, i])
            publish_pose_cmd(pos_pub_obj2, att_pub_obj2, RobotOS.now(), i, u_filtered)
        else
            publish_pose_cmd(pos_pub_obj, att_pub_obj, RobotOS.now(), i, u_filtered)
        end

        # runtime
        solve_time = (RobotOS.now() - t_now).nsecs/10^9
        print("Iteration: ", i, " Time: ", solve_time, "\n")
        if (solve_time > 0.05) && !initial_run
            print("SOLVE TIME TOO LONG\n")
            break
        end

        Z1[:, i] = x_sol[1]
        Z2[:, i] = x_sol[2]
        U[:, i] = u_sol[1]
        Uf[:, i] = u_filtered

        if plot_motion_plan
            plot_current_motion_plan(x_sol, u_sol, mpc_data.xref, length(x_sol), i)
        end

        # readline()

        rossleep(loop_rate)
    end

    return Z1, Z2, U, Uf
end

# Initialize ROS node
init_node("mpc_node")

# subscriber
current_koopman_state = zeros(45)
Subscriber{Float32MultiArray}("/koopman_state", callback, (current_koopman_state,), queue_size=1)

# publishers
pos_pub = Publisher{PointStamped}("/gcs/setpoint/position",queue_size=1)
att_pub = Publisher{QuaternionStamped}("/gcs/setpoint/attitude",queue_size=1)

pos_pub2 = Publisher{PointStamped}("/gcs/setpoint/position2",queue_size=1)
att_pub2 = Publisher{QuaternionStamped}("/gcs/setpoint/attitude2",queue_size=1)

# Load dynamics
mat_file = matopen("/home/oem/flyingSysID/deflated_sysID_nice-resample_dt0-05_history-size-5.mat")
A_full = read(mat_file, "A_full")
B_full = read(mat_file, "B_full")
close(mat_file)

# MPC loop
T = 25
T2 = 400
xref, uref = gen_Z_from_tip_ramp(T2, 5) #T2, history_size
# plot_reference(xref, uref)
problem_data = MPC(T, T2, A_full, B_full, xref, uref);
problem_data.solver.options.max_iterations = 4
problem_data.solver.options.max_dual_updates = 2
print("Problem data initialized\n")

# display(plot(U[1,:], U[2,:], linewidth=2, label="control", aspect_ratio=:equal))
# display(plot(Z[1,:], Z[2,:], linewidth=2, label="position", aspect_ratio=:equal))
# display(plot(X[1,:], X[2,:], linewidth=2, label="drone", aspect_ratio=:equal))

function plot_results(show_control=true, show_reference=true, show_drone=false)
    h = plot([0],[0])
    plot!(Z2[7,:], Z2[8,:], linewidth=2, label="Z2 tip", aspect_ratio=:equal)
    
    if show_drone
        plot!(Z2[1,:], Z2[2,:], linewidth=2, label="Z2 drone", aspect_ratio=:equal)
    end
    
    if show_control
        plot!(U[1,:], U[2,:], linewidth=2, label="control", aspect_ratio=:equal)
        plot!(Uf[1,:], Uf[2,:], linewidth=2, label="filtered control", aspect_ratio=:equal)
    end

    if show_reference
        num_steps = length(Z1[1,:])
        plot!([x[7] for x in xref[1:num_steps]], [x[8] for x in xref[1:num_steps]], linewidth=2, label="tip reference", aspect_ratio=:equal)
    end
    display(h)
end

Z1, Z2, U, Uf = loop(pos_pub, att_pub, pos_pub2, att_pub2, problem_data, current_koopman_state, num_steps=190, override=false, initial_run=true);


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


# [current_koopman_state[1:9] current_koopman_state[10:18] current_koopman_state[19:27] current_koopman_state[28:36] current_koopman_state[37:45]]