import numpy as np

class Simulation:
    def __init__(self, lf, lr, mass, Iz, dt, integrator="euler", model="kinematic"):
        """
        Initialize the simulation parameters.
        """
        self.l_f = lf                   # Distance to front axle (m)
        self.l_r = lr                   # Distance to rear axle (m)
        self.l_wb = lf + lr
        self.mass = mass                # Vehicle mass (kg)
        self.I_z = Iz                   # Yaw moment of inertia (kg*m^2)
        self.dt = dt                    # Time step (s)
        self.sinusoidal_steer = False    # Sinusoidal steer (True/False)
        self.integrator = integrator    # Integrator choice
        self.model = model              # Model choice
        
        # Aerodynamic and rolling resistance parameters
        self.rho = 1.225               # Air density (kg/m^3)
        self.C_d = 0.3                 # Drag coefficient (typical for cars)
        self.A = 2.2                   # Frontal area (m^2)
        self.C_rr = 0.015              # Rolling resistance coefficient

        # Initialize states
        self.x = 0                      # X position (m)
        self.y = 0                      # Y position (m)
        self.theta = 0                  # Heading angle (rad)
        self.vx = 24.0                  # Longitudinal velocity (m/s)
        self.vy = 0                     # Lateral velocity (m/s)
        self.r = 0                      # Yaw rate (rad/s)

        # Pacejka's Magic Formula coefficients
        self.B, self.C, self.D, self.E = 7.1433, 1.3507, 1.0489, -0.0074722
        self.B_front, self.C_front, self.D_front, self.E_front = self.B, self.C, self.D, self.E
        self.B_rear, self.C_rear, self.D_rear, self.E_rear = self.B, self.C, self.D, self.E
        
        # Cornering stiffness front/rear (N/rad)
        self.cornering_stiffness_front = self.B_front*self.C_front*self.D_front
        self.cornering_stiffness_rear = self.B_rear*self.C_rear*self.D_rear  

        # The following states are kept for plotting, keeping these values is not needed throughout the simulation
        self.alpha_front = 0
        self.alpha_rear = 0
        self.Fy_front = 0
        self.Fy_rear = 0
        self.beta = 0

    def kinematic_model(self, ax, delta):
        """ Kinematic single-track model equations of motion. """
        
        # Total longitudinal force including aerodynamic drag and rolling resistance forces
        F_aero = 0.5 * self.rho * self.C_d * self.A * (self.vx**2)
        F_roll = self.C_rr * self.mass * 9.81
        Fx = ax * self.mass - (F_aero + F_roll)

        # Side slip angle
        self.beta = np.arctan(self.vy / self.vx)
        
        # The array is in the format [dx, dy, dtheta, dvx, dvy, dr]
        dx = np.array([
            self.vx * np.cos(self.theta) - self.vy * np.sin(self.theta),    
            self.vx * np.sin(self.theta) + self.vy * np.cos(self.theta),    
            self.vx * np.tan(delta) / self.l_wb,                            
            (1/self.mass) * Fx + self.r * self.vy,      
            0,                                                              
            0                                                               
        ])

        return dx

    def linear_single_track_model(self, ax, delta):
        """ Linear single-track model with aerodynamic and rolling resistance. """

        # Total longitudinal force including aerodynamic drag and rolling resistance forces
        F_aero = 0.5 * self.rho * self.C_d * self.A * (self.vx**2)
        F_roll = self.C_rr * self.mass * 9.81
        Fx = ax * self.mass - (F_aero + F_roll)

        # Vertical forces (nominal vertical load)
        F_n = self.mass * 9.81
        Fz_front_nominal = (self.l_r / self.l_wb) * F_n
        Fz_rear_nominal = (self.l_f / self.l_wb) * F_n

        # Side slip angle
        self.beta = np.arctan(self.vy / self.vx)

        # Front and rear lateral forces computed based on the tire slip angles
        # This is the linear part (valid just for small slip angles)
        self.alpha_front = delta - (self.vy + self.l_f * self.r) / self.vx
        self.alpha_rear = - (self.vy - self.l_r * self.r) / self.vx
        self.Fy_front = Fz_front_nominal * self.cornering_stiffness_front * self.alpha_front
        self.Fy_rear = Fz_rear_nominal * self.cornering_stiffness_rear * self.alpha_rear

        # Dynamics equations
        # The array is in the format [dx, dy, dtheta, dvx, dvy, dr]
        dx = np.array([
            self.vx * np.cos(self.theta) - self.vy * np.sin(self.theta),              
            self.vx * np.sin(self.theta) + self.vy * np.cos(self.theta),               
            self.r,                                                                    
            (1/self.mass) * (Fx - self.Fy_front * np.sin(delta)) + self.vy * self.r,               
            (1/self.mass) * (self.Fy_rear + self.Fy_front * np.cos(delta)) - self.r * self.vx,    
            (1/self.I_z) * (self.Fy_front * self.l_f * np.cos(delta) - self.Fy_rear * self.l_r)   
        ])
        
        return dx

    def nonlinear_single_track_model(self, ax, delta):
        """ Nonlinear single-track model with aerodynamic and rolling resistance. """

        # Total longitudinal force including aerodynamic drag and rolling resistance forces
        F_aero = 0.5 * self.rho * self.C_d * self.A * (self.vx**2)
        F_roll = self.C_rr * self.mass * 9.81
        Fx = ax * self.mass - (F_aero + F_roll)

        # Vertical forces (nominal vertical load)
        F_n = self.mass * 9.81
        Fz_front_nominal = (self.l_r / self.l_wb) * F_n
        Fz_rear_nominal = (self.l_f / self.l_wb) * F_n

        # Side slip angle
        self.beta = np.arctan(self.vy / self.vx)

        # Front and rear lateral forces computed based on the tire slip angles
        # This is the non-linear part, used to represent lateral forces beyond the "small slip angle" region
        # Please note: in order to make the model works for small velocities too, a minimum value of 0.5 has been set for the denominator
        self.alpha_front = delta - np.arctan((self.vy + self.l_f * self.r) / max(0.5, self.vx))
        self.alpha_rear = - np.arctan((self.vy - self.l_r * self.r) / max(0.5, self.vx))
        self.Fy_front = Fz_front_nominal * self.D_front * np.sin(self.C_front * np.arctan(self.B_front * self.alpha_front - self.E_front * (self.B_front * self.alpha_front - np.arctan(self.B_front * self.alpha_front))))
        self.Fy_rear = Fz_rear_nominal * self.D_rear * np.sin(self.C_rear * np.arctan(self.B_rear * self.alpha_rear - self.E_rear * (self.B_rear * self.alpha_rear - np.arctan(self.B_rear * self.alpha_rear))))

        # Dynamics equations
        # The array is in the format [dx, dy, dtheta, dvx, dvy, dr]
        dx = np.array([
        self.vx * np.cos(self.theta) - self.vy * np.sin(self.theta),    
            self.vx * np.sin(self.theta) + self.vy * np.cos(self.theta),   
            self.r,                                                                
            (1/self.mass) * (Fx - self.Fy_front * np.sin(delta)) + self.vy * self.r,   
            (1/self.mass) * (self.Fy_rear + self.Fy_front * np.cos(delta)) - self.r * self.vx, 
            (1/self.I_z) * (self.Fy_front * self.l_f * np.cos(delta) - self.Fy_rear * self.l_r) 
        ])
        
        return dx

    def integrate(self, ax, delta):
        """ Select the integrator method and apply it to update the state. """
        if self.integrator == "euler":
            self.euler_step(ax, delta)
        elif self.integrator == "rk4":
            self.rk4_step(ax, delta)

    def euler_step(self, ax, delta):
        """ Euler integration method. """
        dx = self.compute_dx(ax, delta)
        self.update_state(dx)

    def rk4_step(self, ax, delta):
        """ Runge-Kutta 4th order integration method. """
        k1 = self.compute_dx(ax, delta)
        self.update_state(k1, scale=0.5)
        
        k2 = self.compute_dx(ax, delta)
        self.update_state(k2, scale=0.5, revert=k1, revert_scale=0.5)
        
        k3 = self.compute_dx(ax, delta)
        self.update_state(k3, scale=1, revert=k2, revert_scale=0.5)

        k4 = self.compute_dx(ax, delta)
        self.update_state(np.zeros(6), scale=1, revert=k3, revert_scale=1)
        
        # Combine k1, k2, k3, k4 for RK4 update
        dx = (k1 + 2*k2 + 2*k3 + k4) / 6
        self.update_state(dx)

    def compute_dx(self, ax, delta):
        """ Compute the state derivatives using the chosen model. """
        if self.model == "kinematic":
            return self.kinematic_model(ax, delta)
        elif self.model == "linear":
            return self.linear_single_track_model(ax, delta)
        elif self.model == "nonlinear":
            return self.nonlinear_single_track_model(ax, delta)

    def update_state(self, dx, scale=1, revert=None, revert_scale=1):
        """ Update state with scaled dx. Optionally revert previous state for RK4. """
        if revert is not None:
            self.x -= revert[0] * self.dt * revert_scale
            self.y -= revert[1] * self.dt * revert_scale
            self.theta -= revert[2] * self.dt * revert_scale
            self.vx -= revert[3] * self.dt * revert_scale
            self.vy -= revert[4] * self.dt * revert_scale
            self.r -= revert[5] * self.dt * revert_scale

        self.x += dx[0] * self.dt * scale
        self.y += dx[1] * self.dt * scale
        self.theta += dx[2] * self.dt * scale
        self.vx += dx[3] * self.dt * scale
        self.vy += dx[4] * self.dt * scale
        self.r += dx[5] * self.dt * scale
