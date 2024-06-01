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

function loop(pos_pub_obj, att_pub_obj, pos_pub_obj2, att_pub_obj2, offset_pub, mpc_data, current_koopman_state; alpha = .2, num_steps = 200, override=true, plot_motion_plan=false, initial_run=false, use_kf_offset=0)
    i = 0
    
    x_sol, u_sol = mpc_data.x_sol, mpc_data.u_sol

    num_states = length(x_sol[1])
    
    U = zeros(3, num_steps)
    Uf = zeros(3, num_steps)
    Z1 = zeros(num_states, num_steps)
    Z2 = zeros(num_states, num_steps)

    x_pred = zeros(num_states)
    
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

    reset!(mpc_data.kalman_filter)
    
    loop_rate = Rate(20.0)
    for i in 1:num_steps
        t_now = RobotOS.now()

        if i > 1
            x_pred = kalman_filter_step!(mpc_data.kalman_filter, x_sol[2], current_koopman_state[1:9])
        end
        
        if ! override
            x_sol[2][1:45] .= current_koopman_state
            x_sol[2][46:54] .= use_kf_offset*x_pred[46:54]
        end

        if ! safety_check(current_koopman_state)
            break
        end

        # print("State: ", round.(x_sol[2][1:3]; digits=4), "\n")
        # update_problem!(mpc_data.solver, mpc_data.objectives[i], mpc_data.model, x_sol, [u_sol[2:end]..., u_sol[end]])
        update_problem!(mpc_data.solver, mpc_data.objectives[i], mpc_data.model, x_sol, mpc_data.uref[(i-1) .+ (1:length(mpc_data.model))])
        
        x_sol, u_sol = get_trajectory(mpc_data.solver)

        u_filtered = alpha * u_sol[1] + (1 - alpha) * u_filtered

        if override
            publish_pose_cmd(pos_pub_obj, att_pub_obj, RobotOS.now(), i, df[1:3, i])
            publish_pose_cmd(pos_pub_obj2, att_pub_obj2, RobotOS.now(), i, u_filtered)
        else
            publish_pose_cmd(pos_pub_obj, att_pub_obj, RobotOS.now(), i, u_filtered)
        end

        # publish estimate of koopman model offset
        offset_pub_msg = Float32MultiArray()
        offset_pub_msg.data = x_pred[46:54]
        publish(offset_pub, offset_pub_msg)

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

offset_pub = Publisher{Float32MultiArray}("/koopman_model_offset",queue_size=1)

# Load dynamics
mat_file = matopen("/home/oem/flyingSysID/deflated_sysID_nice-resample_dt0-05_history-size-5.mat")
# mat_file = matopen("../flyingSysID/deflated_sysID_nice-resample_dt0-05_history-size-5.mat")
A_full = read(mat_file, "A_full")
B_full = read(mat_file, "B_full")
close(mat_file)

A_extended = [A_full zeros(45, 9); zeros(9, 45) I(9)]
A_extended[1:9, 46:54] = I(9)
B_extended = [B_full; zeros(9, 3)]
Q_coeff=0.1
R_coeff=1000 #larger for slower-changing signal
KF = KalmanFilter(Matrix(A_extended), B_extended; Q_coeff=Q_coeff, R_coeff=R_coeff)
# MPC horizon and runtime
T = 25

# 4 SECOND PERIOD
# T2 = 199
# num_steps = 157
# xref, uref = gen_Z_from_tip_ramp(T2, 5; initial_period=300, final_period=4, ramp_duration=4) #T, history_size
# df = CSV.read("/home/oem/StanfordMSL/TrajBridge/src/bridge_px4/trajectories/EE_fig8_4s_ramp_MPCref.csv", DataFrame)

# 6 SECOND PERIOD
# T2 = 
# xref, uref = gen_Z_from_tip_ramp(T2, 5; initial_period=300, final_period=6, ramp_duration=5) #T, history_size
# df = CSV.read("/home/oem/StanfordMSL/TrajBridge/src/bridge_px4/trajectories/EE_fig8_6s_ramp_MPCref.csv", DataFrame)

# 10 SECOND PERIOD
T2 = 313
num_steps = 285
xref, uref = gen_Z_from_tip_ramp(T2, 5; initial_period=300, final_period=10, ramp_duration=5) #T, history_size
df = CSV.read("/home/oem/StanfordMSL/TrajBridge/src/bridge_px4/trajectories/EE_fig8_10s_ramp_MPCref.csv", DataFrame)
# df = CSV.read("src/bridge_px4/trajectories/EE_fig8_10s_ramp_MPCref.csv", DataFrame)

# LONG HORIZON TEST
# T2 = 999
# num_steps = 999-26
# xref, uref = gen_Z_from_tip_ramp(T2, 5; initial_period=300, final_period=6, ramp_duration=5) #T, history_size
# df = CSV.read("/home/oem/StanfordMSL/TrajBridge/src/bridge_px4/trajectories/EE_fig8_6s_ramp_OLref_1ksteps.csv", DataFrame)

# 10 SECOND HOVER
# T2 = 300
# num_steps = 270
# xref = [gen_z_from_tip([0.0, 0.0, 0.5], 45) for i in 1:T2]
# uref = [x[1:3] for x in xref]

# update xref
xref = [[x; zeros(9)] for x in xref]

# TOGGLE THIS ON OR OFF
uref = [x[1:3] for x in eachcol(df)]

# plot_reference(xref, uref)
# problem_data = MPC(T, T2, A_full, B_full, xref, uref);
# problem_data = MPC(T, T2, A_extended, B_extended, xref, uref, KF; tip_cost = [1.0, 1.0, 30.0], u_cost = [4.0, 4.0, 1.0]);
problem_data = MPC(T, T2, A_extended, B_extended, xref, uref, KF; tip_cost = [1.0, 1.0, 3.0], u_cost = [4.0, 4.0, 1.0]);
problem_data.solver.options.max_iterations = 4
problem_data.solver.options.max_dual_updates = 2
print("Problem data initialized\n")

Z1, Z2, U, Uf = loop(pos_pub, att_pub, pos_pub2, att_pub2, offset_pub, problem_data, current_koopman_state, alpha = .9, num_steps=num_steps, override=false, initial_run=true, use_kf_offset=0);
# Z1, Z2, U, Uf = loop(pos_pub, att_pub, pos_pub2, att_pub2, problem_data, current_koopman_state, num_steps=270, override=false, initial_run=true);

# change_costs!(problem_data, T2; tip_cost = [1.0, 1.0, 30.0], u_cost = [10.0, 10.0, 1.0]);


# test code for KF
# update_problem!(problem_data.solver, problem_data.objectives[1], problem_data.model, problem_data.x_sol, problem_data.uref[1:length(problem_data.model)])
# x_sol, u_sol = get_trajectory(problem_data.solver)
# x_pred = kalman_filter_step!(KF, x_sol[1], x_sol[2][1:9])