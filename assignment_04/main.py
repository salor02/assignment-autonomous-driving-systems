import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from simulation import Simulation

def plot_comparison(results, labels, title, xlabel, ylabel):
    """ Plot comparison of results for a specific state variable. """
    plt.figure(figsize=(10, 6))
    for i, result in enumerate(results):
        plt.plot(result, label=labels[i])
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(True)
    plt.savefig("./img/" + title + ".png", dpi=300, bbox_inches='tight')

def plot_trajectory(x_vals, y_vals, labels):
    """ Plot 2D trajectory (x vs y) for all simulation configurations. """
    plt.figure(figsize=(10, 6))
    for i in range(len(x_vals)):
        plt.plot(x_vals[i], y_vals[i], label=labels[i])
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
    # Vehicle parameters
    lf = 1.156          # Distance from COG to front axle (m)
    lr = 1.42           # Distance from COG to rear axle (m)
    mass = 1200         # Vehicle mass (kg)
    Iz = 1792           # Yaw moment of inertia (kg*m^2)

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
        "Fy_rear": [] # rear tires lateral force
    }

    # Max steer and frequency for sinusoidal steer commands
    steer_max = 0.1
    frequency = 0.5

    for step in range(steps):

        # Make one step simulation via model integration
        # Calculate sinusoidal steering angle
        if sim.sinusoidal_steer:
            time = step * dt
            steer = steer_max * np.sin(2 * np.pi * frequency * time)  # Sinusoidal steering angle

        sim.integrate(ax, steer)
        
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

    return results

def main():
    # Simulation parameters
    dt = 0.08       # Time step (s)
    ax = 1.0            # Constant longitudinal acceleration (m/s^2)
    steer = 0.055         # Constant steering angle (rad)
    sim_time = 10.0      # Simulation duration in seconds
    steps = int(sim_time / dt)  # Simulation steps (30 seconds)

    # List of configurations
    configs = [
        ("rk4", "kinematic"),
        ("rk4", "linear"),
        ("rk4", "nonlinear"),
        ("euler", "kinematic"),
        ("euler", "linear"),
        ("euler", "nonlinear")
    ]

    # Run each simulation and store the results
    all_results = []
    labels = []

    # Each results is an array of subsequent states of a variable, for example in result[0] there will be the array corresponding
    # to the X coord. Then, in all_results will be a matrix of arrays: one line for each simulation and one cell for each subsequent values
    # of a state in that simulation.
    for integrator, model in configs:
        results = run_simulation(ax, steer, dt, integrator, model, steps)
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

    # Plot comparisons for each simulation value
    plot_trajectory(x_results, y_results, labels)
    plot_comparison(theta_results, labels, "Heading Angle Comparison", "Time Step", "Heading Angle (rad)")
    plot_comparison(vx_results, labels, "Longitudinal Velocity Comparison", "Time Step", "Velocity (m/s)")
    plot_comparison(vy_results, labels, "Lateral Velocity Comparison", "Time Step", "Lateral Velocity (m/s)")
    plot_comparison(r_results, labels, "Yaw Rate Comparison", "Time Step", "Yaw Rate (rad/s)")
    plot_comparison(alpha_front_results, labels, "Front Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Front")
    plot_comparison(alpha_rear_results, labels, "Rear Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Rear")
    plot_comparison(steer_results, labels, "Steering Angle Comparison", "Time Step", "Steering Angle (rad)")
    plot_comparison(beta_results, labels, "Side Slip Angle Comparison", "Time Step", "Side Slip Angle (rad)")
    plot_lateral_force(Fy_front_results, alpha_front_results, labels, "Front")
    plot_lateral_force(Fy_rear_results, alpha_rear_results, labels, "Rear")

if __name__ == "__main__":
    main()
