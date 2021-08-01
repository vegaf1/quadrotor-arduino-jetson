using Altro
using TrajectoryOptimization
using RobotDynamics
using StaticArrays, LinearAlgebra
const TO = TrajectoryOptimization
const RD = RobotDynamics

struct DubinsCar <: AbstractModel end
RD.state_dim(::DubinsCar) = 3
RD.control_dim(::DubinsCar) = 2

function RD.dynamics(::DubinsCar,x,u)
    ẋ = @SVector [u[1]*cos(x[3]),
                  u[1]*sin(x[3]),
                  u[2]]
end

model = DubinsCar()
n,m = size(model)    # get state and control dimension
N = 101              # number of time steps (knot points). Should be odd.
tf = 3.0             # total time (sec)
dt = tf / (N-1)      # time step (sec)

model = DubinsCar()
n,m = size(model)    # get state and control dimension
N = 101              # number of time steps (knot points). Should be odd.
tf = 3.0             # total time (sec)
dt = tf / (N-1)      # time step (sec)

x0 = SA_F64[0,0,0]   # start at the origin
xf = SA_F64[1,2,pi]  # goal state


Q  = Diagonal(SA[0.1,0.1,0.01])
R  = Diagonal(SA[0.01, 0.1])
Qf = Diagonal(SA[1e2,1e2,1e3])
obj = LQRObjective(Q,R,Qf,xf,N)

cons = ConstraintList(n,m,N)

# Goal constraint
goal = GoalConstraint(xf)
add_constraint!(cons, goal, N)

# Workspace constraint
bnd = BoundConstraint(n,m, x_min=[-0.1,-0.1,-Inf], x_max=[5,5,Inf])
add_constraint!(cons, bnd, 1:N-1)

# Obstacle Constraint
obs = CircleConstraint(n, SA_F64[1,2], SA_F64[1,1], SA[0.2, 0.3])
add_constraint!(cons, bnd, 1:N-1)

prob = Problem(model, obj, xf, tf, x0=x0, constraints=cons, integration=RK4)

initial_controls!(prob, [@SVector rand(m) for k = 1:N-1])
rollout!(prob)   # simulate the system forward in time with the new controls

solver = ALTROSolver(prob)

# Set up solver options
opts = SolverOptions()
opts.cost_tolerance = 1e-5

# Create a solver, adding in additional options
solver = ALTROSolver(prob, opts, show_summary=false)

solve!(solver)

status(solver)

println("Number of iterations: ", iterations(solver))
println("Final cost: ", cost(solver))
println("Final constraint satisfaction: ", max_violation(solver))
