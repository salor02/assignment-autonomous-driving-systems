from cubic_spline_planner import *
from casadi import *
from casadi.tools import *

# MPC time
T =  1 # Horizon length in seconds
dt = 0.05 # Horizon timesteps
N = int(T/dt) # Horizon total points

max_steer = 3.14  # Maximum steering angle in radians
min_steer = -3.14  # Minimum steering angle in radians

x_error_gain = 50 # weight on x error
y_error_gain = 50 # weight on y error
heading_error_gain = 10 # weight on heading error

# weight for the final prediction step. The last step should weight more than the others
# in order to allow a certain amount of error during the process but minimize the error at the final target position
terminal_cost = 50 

steering_cost = 10000 # this indicates the cost of steering, the higher the more we want to avoid to steer

def casadi_model():
    global F

    # Control
    # Create 1r-1c matrix containing control inputs. 
    # Set steer as element
    u = MX.sym("u",1)
    steer = u[0]

    # Constants - Model parameters
    Lr = 1.42 
    Lf = 1.156
    L = Lf + Lr
    mass = 1200
    Iz = 1792

    #### LINEAR SINGLE TRACK MODEL

    # Pacejka's Magic Formula coefficients
    B, C, D, E = 7.1433, 1.3507, 1.0489, -0.0074722
    B_front, C_front, D_front, E_front = B, C, D, E
    B_rear, C_rear, D_rear, E_rear = B, C, D, E

    # Cornering stiffness front/rear (N/rad)
    cornering_stiffness_front = B_front*C_front*D_front
    cornering_stiffness_rear = B_rear*C_rear*D_rear  

    # State
    x = MX.sym("x",6)
    sx = x[0]   # X Position
    sy = x[1]   # Y Position
    yaw = x[2]  # Yaw Angle
    vx = x[3]   # Longitudinal Velocity
    vy = x[4]   # Lateral Velocity
    r = x[5]    # Yaw Rate

    # Per evitare divisioni per zero se vx è basso in simulazione:
    v_safe = fmax(1.0, vx)
    
    # Slip Angles (Linear approximation for small angles)
    alpha_f = steer - (vy + Lf*r)/v_safe
    alpha_r = - (vy - Lr*r)/v_safe
    
    # Lateral Forces
    FyF = (Lr / L) * mass * 9.81 * cornering_stiffness_front * alpha_f
    FyR = (Lf / L) * mass * 9.81 * cornering_stiffness_rear * alpha_r

    sxdot = v_safe * cos(yaw) - vy * sin(yaw)
    sydot = v_safe * sin(yaw) + vy * cos(yaw)
    yawdot = r
    vxdot = 0.0 # Assumiamo velocità costante nell'orizzonte (gestita dal PID esterno)
    vydot = (FyF * cos(steer) + FyR) / mass - v_safe * r
    rdot  = (Lf * FyF * cos(steer) - Lr * FyR) / Iz

    # Concatenate vertically the expressions creating a row vector
    xdot = vertcat(sxdot, sydot, yawdot, vxdot, vydot, rdot)

    # #### KINEMATIC MODEL
    # # State
    # x = MX.sym("x",4)
    # sx    = x[0]  # position x
    # sy    = x[1]  # position y
    # yaw   = x[2]  # yaw
    # speed = x[3]

    # # ODE right hand side
    # sxdot    = speed*cos(yaw)
    # sydot    = speed*sin(yaw)
    # yawdot   = (speed/L)*tan(steer)
    # speeddot = 0.0

    # # Concatenate vertically the expressions creating a row vector
    # xdot = vertcat(sxdot, sydot, yawdot, speeddot)

    # ODE right hand side function
    # as input are used the state and control inputs,
    # and as output the row vector containing the model expressions
    f = Function('f', [x,u],[xdot])

    # Integrate the step with Explicit Euler
    IN = 1
    xj = x
    for i in range(IN):
        fj = f(xj,u)
        xj += dt*fj/IN

    # Discrete time dynamics function
    F = Function('F', [x,u],[xj])

def opt_step(target, state):
    global F

    # Control for all segments
    nu = N #number of states
    #steer control input (this vector contains the nu states representing the optimal input steer at each step) 
    Us = MX.sym("U",nu) 

    # Initial conditions
    x0 = [state.x, state.y, state.theta, state.vx, state.vy, state.r] # linear single track
    # x0 = [state.x, state.y, state.theta, state.vx] #kinematic
    X0 = MX(x0) # vector containing the initial state

    J = 0 # objective function that should be minimized by the nlp solver

    # build graph
    X=X0
    G = None
    lbg = []
    ubg = []

    # For every temporal step do the following:
    # - integrate the step, by putting the steering value at step k (still unknown) in the defined model
    # - update the cost function giving different weight to each state. A position distance
    #   from target is not penalized as much as a heading error. By this settings we aim to achieve a smooth
    #   drive
    for k in range(nu):
        X = F(X, vertcat(Us[k])) #integration step
        gain_mult = 1
        # give more importance to the last step using a bigger gain
        if(k == nu-1):
            gain_mult=terminal_cost #You can use this multiplier as terminal cost
        J += x_error_gain*gain_mult*(X[0]-target[k][0])**2 #x error cost 
        J += y_error_gain*gain_mult*(X[1]-target[k][1])**2 #y error cost
        J += heading_error_gain*gain_mult*(X[2] - target[k][2])**2 #heading error cost
    # G = X[index] #if you want to set a state to constrain in arg["lbg"] and arg["ubg"]. It can be ignored

    # Objective function and constraints
    J += mtimes(Us.T,Us)*steering_cost * X[3] #The more the speed the more the steering penalty

    # NLP
    nlp = {'x':vertcat(Us), 'f':J}
    
    # Allocate an NLP solver
    opts = {
    "ipopt.tol": 1e-3,
    "ipopt.acceptable_tol": 1e-2,
    "ipopt.acceptable_iter": 5,
    "ipopt.max_iter": 20,
    "ipopt.print_level": 1,
    "expand": True,
    }

    solver = nlpsol("solver", "ipopt", nlp, opts)
    arg = {}

    # Bounds on u and initial condition
    arg["lbx"] =  vertcat(min_steer*np.ones(nu)) # lower bound for steer
    arg["ubx"] =  vertcat(max_steer*np.ones(nu)) # upper bound for steer
    arg["x0"] =    0.0 # first guess 

    #These can be ingnored 
    # # Bounds on g
    # arg["lbg"] = lower_bound on a state
    # arg["ubg"] = inf

    # Solve the problem
    res = solver(**arg)
    #print "f:", res["f"]
    ctrls = reshape(res["x"], (nu,1)).T #reshape to have a row for each step

    return ctrls[0][0]