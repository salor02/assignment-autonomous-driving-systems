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
import frenet_optimal_trajectory as fp
import time 

# Simulation parameters
dt = 0.05         # Time step (s)
ax = 0.0            # Constant longitudinal acceleration (m/s^2)
steer = 0.0      # Constant steering angle (rad)
sim_time = 65    # Simulation duration in seconds
steps = int(sim_time / dt)  # Simulation steps

# Control references
target_speed = 31

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

#Planning
# obstacle lists
ob = np.array([[100.0, -0.5],
                [400.0, 0.5],
                [570.0, 29.0],
                [600.0,100.0],
                [120.0, 200.0],
                [33.0, 200.0],
                [-70.0, 171.0],
                [-100.0, 100.0]
                ])

def load_path(file_path):
    xs = []
    ys = []
    with open(file_path, "r") as f:
        for i, line in enumerate(f, start=1):
            line = line.strip()
            if not line or line.startswith('#'):  # Salta righe vuote o commenti
                continue
                
            parts = [p.strip() for p in line.split(',')]
            if len(parts) != 2:
                print(f"Riga {i} malformata: {repr(line)}")
                continue
                
            try:
                xs.append(float(parts[0]))
                ys.append(float(parts[1]))
            except ValueError as e:
                print(f"Riga {i} non numerica: {parts} -> {e}")
                continue
                
    print(f"Caricati {len(xs)} punti validi")
    return xs, ys

# Load path and create a spline
xs, ys = load_path("oval_trj.txt")
path_spline = cubic_spline_planner.Spline2D(xs, ys)

def point_transform(trg, pose, yaw):

    local_trg = [trg[0] - pose[0], trg[1] - pose[1]]

    return local_trg

def lateral_error_calc(trg, pose, yaw):
    dx = trg[0] - pose[0]
    dy = trg[1] - pose[1]

    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    x_local =  cos_y * dx + sin_y * dy
    y_local = -sin_y * dx + cos_y * dy

    return [x_local, y_local]

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

def plot_time(results, labels, title, xlabel, ylabel):
    """ Plot comparison of results for a specific state variable. """
    plt.figure(figsize=(10, 6))
    for i, result in enumerate(results):
        plt.plot(result, label=labels[i])
        mean = np.mean(result)

    plt.axhline(y=mean, color='r', linestyle='--', label=f'Mean Execution Time: {mean:.2f} ms')

    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(True)
    plt.savefig("./img/" + title + ".png", dpi=300, bbox_inches='tight')

def plot_trajectory(x_vals, y_vals, labels, path_spline, frenet_x_results, frenet_y_results):
    """ Plot 2D trajectory (x vs y) for all simulation configurations and path_spline trajectory. """
    plt.figure(figsize=(10, 6))
    
    # Plot the simulation trajectories
    for i in range(len(x_vals)):
        plt.plot(x_vals[i], y_vals[i], label=labels[i])

    # Plot the frenet planner trajectory
    for i in range(len(frenet_x_results)):
        plt.plot(frenet_x_results[i], frenet_y_results[i], label="Frenet Spline", linestyle="--", color="green")

    # Plot the path_spline trajectory
    spline_x = [path_spline.calc_position(s)[0] for s in np.linspace(0, path_spline.s[-1], 1000)]
    spline_y = [path_spline.calc_position(s)[1] for s in np.linspace(0, path_spline.s[-1], 1000)]
    plt.plot(spline_x, spline_y, label="Path Spline", linestyle="--", color="red")

    # Plot obstacles
    if(len(ob[0]) != 0):
        plt.scatter(ob[:, 0], ob[:, 1], c='black', label="Obstacles", marker='x')
    
    # Customize plot
    plt.title("2D Trajectory Comparison")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.savefig("./img/Trajectory.png", dpi=300, bbox_inches='tight')
    plt.show()

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
        "lateral_error": [], # position error w.r.t. reference path
        "frenet_x": [], # next x position of the planned path
        "frenet_y": [], # next y position of the planned path
        "frenet_lateral_error": [], # position error w.r.t. planned path
        "frenet_execution_time": [] # execution time of the frenet planner
    }

    casadi_model()

    # Settling time variables
    settling_treshold = target_speed * 0.05
    settling_timestep = 0
    settled = False

    # states for Frenet-planner
    c_speed = 0.0  # current speed [m/s]
    c_accel = 0.0  # current acceleration [m/ss]
    c_d = 0.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    for step in range(steps):
    
        # Print time
        print("Time:", step*dt)

        # Calculate ax to track speed
        ax, velocity_error = long_control_pid.compute(target_speed, sim.vx, dt)

        # This mechanism allows to keep track of the settling time
        if not settled:
            if abs(velocity_error) <= settling_treshold:
                settling_timestep = step
                settled = True
        else:
            if abs(velocity_error) > settling_treshold:
                settled = False

        ### FRENET-PLANNER
        start_time = time.perf_counter()  # start time measurement
        
        frenet_path = fp.frenet_optimal_planning(path_spline, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob)
        
        end_time = time.perf_counter()    # end time measurement
        frenet_execution_time = (end_time - start_time) * 1000 # conversion in ms
        
        if(frenet_path is None):
            print("None available paths found from Frenet...")
            break

        frenetpath_spline = cubic_spline_planner.Spline2D(frenet_path.x, frenet_path.y)

        # The following lines compute the projection of the current vehicle position on the path. Keep in mind that
        # the vehicle has a global position <x,y>. By the following code we are looking for the position of the vehicle position_projected
        # on the path. "If the vehicle was following the path perfectly, where would it be now?"
        actual_position = sim.x, sim.y # actual position of the vehicle in global coords
        path_spline.update_current_s(actual_position) # update the cur_s longitudinal coord in the reference spline
        frenetpath_spline.update_current_s(actual_position) # update the cur_s longitudinal coord in the planned spline

        ## LATERAL ERROR CHECK
        # Compute the distance between the actual position and the projection point in order to get the lateral error (Y coord)
        
        # get actual position projected on the path/spline
        global_position_projected = path_spline.calc_position(path_spline.cur_s) # get the projection of the vehicle on the spline (in global coords)
        prj = [ global_position_projected[0], global_position_projected[1] ] # this is the point where the vehicle would be on the path in the ideal case
        local_error = lateral_error_calc(prj, actual_position, sim.theta)

        # if(abs(local_error[1]) > 4.0):
        #     print("Lateral error is higher than 4.0... ending the simulation")
        #     print("Lateral error: ", local_error[1])
        #     break

        local_position_projected = frenetpath_spline.calc_position(frenetpath_spline.cur_s)
        prj = [ local_position_projected[0], local_position_projected[1] ]
        frenetlocal_error = lateral_error_calc(prj, actual_position, sim.theta)

        # The following lines look for the point belonging the the frenet path which is the closest to the current
        # position of the vehicle based on the reference path longitudinal coordinate. This is done in order to
        # create a new frenet curve that's perfectly "attached" to the previous one.
        nearest_idx = 0
        nearest_distance = abs(path_spline.cur_s - frenet_path.s[0])
        for i in range(len(frenet_path.s)):
            dist = abs(path_spline.cur_s - frenet_path.s[i])
            if(dist < nearest_distance):
                nearest_distance = dist
                nearest_idx = i

        # Initialize values for the next Frenet iteration
        s0 = frenet_path.s[nearest_idx]
        c_d = frenet_path.d[nearest_idx]
        c_d_d = frenet_path.d_d[nearest_idx]
        c_d_dd = frenet_path.d_dd[nearest_idx]
        c_speed = frenet_path.s_d[nearest_idx]
        c_accel = frenet_path.s_dd[nearest_idx]

        # ### PURE PURSUIT

        # # The following lines compute the target position in global coords, based on the lookahead distance
        # # Bonus: include here the curvature dependency
        # Lf = k_pp * sim.vx + look_ahead # compute the lookahead distance, depending on the current velocity (the look_ahead variable is the minimum value of Lf)
        # # TO-DO: Extend it to depend on curvature and/or speed
        # s_pos = frenetpath_spline.cur_s + Lf # add calculated distance to the current longitudinal position
        # trg = frenetpath_spline.calc_position(s_pos) # get global target coords <x,y>

        # # Adjust CoG position to the rear axle position for PP
        # pp_position = actual_position[0] + lr * math.cos(sim.theta), actual_position[1] + lr * math.sin(sim.theta)

        # # Compute distance between vehicle rear axle and lookahead point, both expressed in global coords
        # target_pos_local = point_transform([trg[0], trg[1]], pp_position, sim.theta)

        # # Calculate steer to track path
        # steer = pp_controller.compute_steering_angle(target_pos_local, sim.theta, Lf)

        # ### STANLEY

        # # Adjust CoG position to the front axle position (convention for Stanley)
        # px_front = local_position_projected[0] + lf * math.cos(sim.theta)
        # py_front = local_position_projected[1] + lf * math.sin(sim.theta)
        # stanley_target = px_front, py_front, frenetpath_spline.calc_yaw(frenetpath_spline.cur_s)
        
        # actual_pose = sim.x, sim.y, sim.theta
        # steer = stanley_controller.compute_steering_angle(actual_pose, stanley_target, sim.vx)

        ### MPC

        # The following lines compute the target point on the path for the future horizon
        targets = [ ]
        s_pos = frenetpath_spline.cur_s
        for i in range(N):
            step_increment = (sim.vx)*dt
            trg = frenetpath_spline.calc_position(s_pos)
            t_yaw = frenetpath_spline.calc_yaw(s_pos)
            trg = [ trg[0], trg[1], t_yaw ]
            targets.append(trg)
            s_pos += step_increment

        steer = opt_step(targets, sim)

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
        results["steer"].append(float(steer))
        results["beta"].append(sim.beta)
        results["Fy_front"].append(sim.Fy_front)
        results["Fy_rear"].append(sim.Fy_rear)
        results["ax"].append(ax)

        # Get Frenet path coords
        results["frenet_x"].append(frenet_path.x[0])
        results["frenet_y"].append(frenet_path.y[0])

        # Errors
        results["velocity_error"].append(velocity_error)
        results["lateral_error"].append(local_error[1])
        results["frenet_lateral_error"].append(frenetlocal_error[1])

        # Frenet execution time
        results["frenet_execution_time"].append(frenet_execution_time)

    return results, settling_timestep

def main():

    # List of configurations
    configs = [
        ("rk4", "nonlinear"),
    ]

    # Run each simulation and store the results
    all_results = []
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
    frenet_x_results = [result["frenet_x"] for result in all_results]
    frenet_y_results = [result["frenet_y"] for result in all_results]
    frenet_lateral_error_results = [result["frenet_lateral_error"] for result in all_results]
    frenet_execution_time_results = [results["frenet_execution_time"] for result in all_results]

    # Plot comparisons for each simulation value
    plot_trajectory(x_results, y_results, labels, path_spline, frenet_x_results, frenet_y_results)
    plot_comparison(theta_results, labels, "Heading Angle Comparison", "Time Step", "Heading Angle (rad)")
    plot_comparison(vx_results, labels, "Longitudinal Velocity Comparison", "Time Step", "Velocity (m/s)", settling_timestep)
    plot_comparison(vy_results, labels, "Lateral Velocity Comparison", "Time Step", "Lateral Velocity (m/s)")
    plot_comparison(r_results, labels, "Yaw Rate Comparison", "Time Step", "Yaw Rate (rad/s)")
    plot_comparison(alpha_front_results, labels, "Front Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Front")
    plot_comparison(alpha_rear_results, labels, "Rear Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Rear")
    plot_comparison(steer_results, labels, "Steering Angle Comparison", "Time Step", "Steering Angle (rad)")
    plot_comparison(beta_results, labels, "Side Slip Angle Comparison", "Time Step", "Side Slip Angle (rad)")
    plot_comparison(ax_results, labels, "Longitudinal Acceleration Comparison", "Time Step", "Acceleration (m/s^2)")
    plot_comparison(velocity_error_results, labels, "Velocity Error", "Time Step", "Velocity Error (m/s)", settling_timestep)
    plot_comparison(lateral_error_results, labels, "Reference Path Lateral Error", "Time Step", "Lateral Error (m)")
    plot_comparison(frenet_lateral_error_results, labels, "Frenet Planned Path Lateral Error", "Time Step", "Lateral Error (m)")
    plot_lateral_force(Fy_front_results, alpha_front_results, labels, "Front")
    plot_lateral_force(Fy_rear_results, alpha_rear_results, labels, "Rear")
    plot_time(frenet_execution_time_results, labels, "Frenet Execution Time", "Time Step", "Execution Time (ms)")

if __name__ == "__main__":
    main()
