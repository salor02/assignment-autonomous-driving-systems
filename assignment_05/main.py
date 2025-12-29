import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from simulation import Simulation
import pid
import purepursuit
import stanley
from mpc import *
import cubic_spline_planner
import math
import matplotlib

# Simulation parameters
dt = 0.05         # Time step (s)
ax = 0.0            # Constant longitudinal acceleration (m/s^2)
steer = 0.0      # Constant steering angle (rad)
sim_time = 170    # Simulation duration in seconds
steps = int(sim_time / dt)  # Simulation steps

# Control references
target_speed =26.2

# Vehicle parameters
lf = 1.156          # Distance from COG to front axle (m)
lr = 1.42           # Distance from COG to rear axle (m)
wheelbase = lf + lr
mass = 1200         # Vehicle mass (kg)
Iz = 1792           # Yaw moment of inertia (kg*m^2)
max_steer = 3.14  # Maximum steering angle in radians

# Create instance of PID for Longitudinal Control
long_control_pid = pid.PIDController(kp=1.6, ki=0.85, kd=0.01, output_limits=(-2, 2))

# Create instance of PurePursuit, Stanley and MPC for Lateral Control
k_pp = 0.1  # Speed proportional gain for Pure Pursuit
look_ahead = 2.0  # Minimum look-ahead distance for Pure Pursuit
k_stanley = 1.9  # Gain for cross-track error for Stanley
pp_controller = purepursuit.PurePursuitController(wheelbase, max_steer)
stanley_controller = stanley.StanleyController(k_stanley, lf, max_steer)

def load_path(file_path):
    file = open(file_path, "r")
    
    xs = []
    ys = []

    while(file.readline()):
        line = file.readline()
        xs.append( float(line.split(",")[0]) )
        ys.append( float(line.split(",")[1]) )
    return xs, ys

# Load path and create a spline
xs, ys = load_path("oval_trj.txt")
path_spline = cubic_spline_planner.Spline2D(xs, ys)

def point_transform(trg, pose, yaw):

    local_trg = [trg[0] - pose[0], trg[1] - pose[1]]

    return local_trg

def plot_comparison(results, labels, title, xlabel, ylabel, settling_timestep = None):
    """ Plot comparison of results for a specific state variable. """
    plt.figure(figsize=(10, 6))
    for i, result in enumerate(results):
        plt.plot(result, label=labels[i])

    if settling_timestep is not None:
        plt.axvline(x=settling_timestep, color='r', linestyle='--', label=f'Settling Time: {settling_timestep*dt:.2f} ms')

    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(True)
    plt.savefig("./img/" + title + ".png", dpi=300, bbox_inches='tight')

def plot_trajectory(x_vals, y_vals, labels, path_spline):
    """ Plot 2D trajectory (x vs y) for all simulation configurations and path_spline trajectory. """
    plt.figure(figsize=(10, 6))
    
    # Plot the simulation trajectories
    for i in range(len(x_vals)):
        plt.plot(x_vals[i], y_vals[i], label=labels[i])
    
    # Plot the path_spline trajectory
    spline_x = [path_spline.calc_position(s)[0] for s in np.linspace(0, path_spline.s[-1], 1000)]
    spline_y = [path_spline.calc_position(s)[1] for s in np.linspace(0, path_spline.s[-1], 1000)]
    plt.plot(spline_x, spline_y, label="Path Spline", linestyle="--", color="red")
    
    # Customize plot
    plt.title("2D Trajectory Comparison")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.savefig("./img/Trajectory.png", dpi=300, bbox_inches='tight')

# This function plots the Fy vs alpha graphs
def plot_lateral_force(Fy, alpha, labels, force_type):
    """ Plot lateral force as a function of slip angle """
    plt.figure(figsize=(10, 6))
    for i, simulation in enumerate(Fy):
        plt.plot(alpha[i], Fy[i], label=labels[i])
    plt.title(force_type + " lateral force as function of slip angle")
    plt.xlabel(force_type + " lateral slip angle (rad)")
    plt.ylabel(force_type + " lateral force (N)")
    plt.legend()
    plt.grid(True)
    plt.savefig("./img/" + force_type + " Lateral Force vs Slip Angle.png", dpi=300, bbox_inches='tight')


def run_simulation(ax, steer, dt, integrator, model, steps=500):
    """ Run a simulation with the given parameters and return all states. """

    # Initialize the simulation
    sim = Simulation(lf, lr, mass, Iz, dt, integrator=integrator, model=model)

    # Storage for state variables and slip angles    
    results = {
        "x": [],
        "y": [],
        "theta": [],
        "vx": [],
        "vy": [],
        "r": [], # yaw rate
        "alpha_front": [], # front tires slip angle
        "alpha_rear": [], # rear tires slip angle
        "steer": [], # steering angle (delta)
        "beta": [], # side slip angle
        "Fy_front": [], # front tires lateral force
        "Fy_rear": [], # rear tires lateral force
        "ax": [], # acceleration
        "velocity_error": [], # error w.r.t. target speed
        "lateral_error": [], # position error w.r.t. path 
    }

    casadi_model()

    # Settling time variables
    settling_treshold = target_speed * 0.05
    settling_timestep = 0
    settled = False

    for step in range(steps):
    
        # Print time
        print("Time:", step*dt)

        # Calculate ax to track speed (Exercise 1)
        ax, velocity_error = long_control_pid.compute(target_speed, sim.vx, dt)
        # steer = 0 # Exercise 1 only

        # This mechanism allows to keep track of the settling time
        if not settled:
            if abs(velocity_error) <= settling_treshold:
                settling_timestep = step
                settled = True
        else:
            if abs(velocity_error) > settling_treshold:
                settled = False

        # The following lines compute the projection of the current vehicle position on the path. Keep in mind that
        # the vehicle has a global position <x,y>. By the following code we are looking for the position of the vehicle position_projected
        # on the path. "If the vehicle was following the path perfectly, where would it be now?"
        actual_position = sim.x, sim.y # actual position of the vehicle in global coords
        path_spline.update_current_s(actual_position) # update the cur_s longitudinal coord in the spline (frenet-frame)
        position_projected = path_spline.calc_position(path_spline.cur_s) # get the projection of the vehicle on the spline (in global coords)
        prj = [ position_projected[0], position_projected[1] ] # this is the point where the vehicle would be on the path in the ideal case

        ### LATERAL ERROR CHECK
        # Compute the distance between the actual position and the projection point in order to get the lateral error (Y coord)
        local_error = point_transform(prj, actual_position, sim.theta)

        if(abs(local_error[1]) > 1.0):
            print("Lateral error is higher than 1.0... ending the simulation")
            print("Lateral error: ", local_error[1])
            break

        ### PURE PURSUIT

        # # The following lines compute the target position in global coords, based on the lookahead distance
        # Lf = k_pp * sim.vx + look_ahead # compute the lookahead distance, depending on the current velocity (the look_ahead variable is the minimum value of Lf)
        # s_pos = path_spline.cur_s + Lf # add calculated distance to the current longitudinal position
        # trg = path_spline.calc_position(s_pos) # get global target coords <x,y>

        # # Adjust CoG position to the rear axle position for PP
        # pp_position = actual_position[0] + lr * math.cos(sim.theta), actual_position[1] + lr * math.sin(sim.theta)

        # # Compute distance between vehicle rear axle and lookahead point, both expressed in global coords
        # target_pos_local = point_transform([trg[0], trg[1]], pp_position, sim.theta)

        # # Calculate steer to track path
        # steer = pp_controller.compute_steering_angle(target_pos_local, sim.theta, Lf)

        ### STANLEY

        # Adjust CoG position to the front axle position (convention for Stanley)
        px_front = position_projected[0] + lf * math.cos(sim.theta)
        py_front = position_projected[1] + lf * math.sin(sim.theta)
        stanley_target = px_front, py_front, path_spline.calc_yaw(path_spline.cur_s)
        
        actual_pose = sim.x, sim.y, sim.theta
        steer = stanley_controller.compute_steering_angle(actual_pose, stanley_target, sim.vx)

        ### MPC

        # # get future horizon targets pose
        # targets = [ ]
        # s_pos = path_spline.cur_s
        # for i in range(N):
        #     step_increment = (sim.vx)*dt
        #     trg = path_spline.calc_position(s_pos)
        #     t_yaw = path_spline.calc_yaw(s_pos)
        #     trg = [ trg[0], trg[1], t_yaw ]
        #     targets.append(trg)
        #     s_pos += step_increment

        # steer = opt_step(targets, sim)

        # Make one step simulation via model integration
        sim.integrate(ax, float(steer))
        
        # Append each state to corresponding list
        results["x"].append(sim.x)
        results["y"].append(sim.y)
        results["theta"].append(sim.theta)
        results["vx"].append(sim.vx)
        results["vy"].append(sim.vy)
        results["r"].append(sim.r)

        # Get other simulation values
        results["alpha_front"].append(sim.alpha_front)
        results["alpha_rear"].append(sim.alpha_rear)
        results["steer"].append(steer)
        results["beta"].append(sim.beta)
        results["Fy_front"].append(sim.Fy_front)
        results["Fy_rear"].append(sim.Fy_rear)
        results["ax"].append(ax)

        # Errors
        results["velocity_error"].append(velocity_error)
        results["lateral_error"].append(local_error[1])

    return results, settling_timestep

def main():

    # List of configurations
    configs = [
        ("rk4", "nonlinear"),
    ]

    # Run each simulation and store the results
    all_results = []
    actual_state = []
    labels = []
    settling_timestep = []

    # Each results is an array of subsequent states of a variable, for example in result[0] there will be the array corresponding
    # to the X coord. Then, in all_results will be a matrix of arrays: one line for each simulation and one cell for each subsequent values
    # of a state in that simulation.
    for integrator, model in configs:
        results, settling_timestep = run_simulation(ax, steer, dt, integrator, model, steps)
        all_results.append(results)
        labels.append(f"{integrator.capitalize()} - {model.capitalize()}")

    # Separate each value for plotting
    x_results = [result["x"] for result in all_results]
    y_results = [result["y"] for result in all_results]
    theta_results = [result["theta"] for result in all_results]
    vx_results = [result["vx"] for result in all_results]
    vy_results = [result["vy"] for result in all_results]
    r_results = [result["r"] for result in all_results]
    alpha_front_results = [result["alpha_front"] for result in all_results]
    alpha_rear_results = [result["alpha_rear"] for result in all_results]
    steer_results = [result["steer"] for result in all_results]
    beta_results = [result["beta"] for result in all_results]
    Fy_front_results = [result["Fy_front"] for result in all_results]
    Fy_rear_results = [result["Fy_rear"] for result in all_results]
    ax_results = [result["ax"] for result in all_results]
    velocity_error_results = [result["velocity_error"] for result in all_results]
    lateral_error_results = [result["lateral_error"] for result in all_results]

    # Plot comparisons for each simulation value
    plot_trajectory(x_results, y_results, labels, path_spline)
    # plot_comparison(theta_results, labels, "Heading Angle Comparison", "Time Step", "Heading Angle (rad)")
    plot_comparison(vx_results, labels, "Longitudinal Velocity Comparison", "Time Step", "Velocity (m/s)", settling_timestep)
    plot_comparison(vy_results, labels, "Lateral Velocity Comparison", "Time Step", "Lateral Velocity (m/s)")
    # plot_comparison(r_results, labels, "Yaw Rate Comparison", "Time Step", "Yaw Rate (rad/s)")
    plot_comparison(alpha_front_results, labels, "Front Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Front")
    plot_comparison(alpha_rear_results, labels, "Rear Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Rear")
    plot_comparison(steer_results, labels, "Steering Angle Comparison", "Time Step", "Steering Angle (rad)")
    plot_comparison(beta_results, labels, "Side Slip Angle Comparison", "Time Step", "Side Slip Angle (rad)")
    plot_comparison(ax_results, labels, "Longitudinal Acceleration Comparison", "Time Step", "Acceleration (m/s^2)")
    plot_comparison(velocity_error_results, labels, "Velocity Error", "Time Step", "Velocity Error (m/s)", settling_timestep)
    plot_comparison(lateral_error_results, labels, "Lateral Error", "Time Step", "Lateral Error (m)")
    plot_lateral_force(Fy_front_results, alpha_front_results, labels, "Front")
    plot_lateral_force(Fy_rear_results, alpha_rear_results, labels, "Rear")

if __name__ == "__main__":
    main()
