#!/usr/bin/env julia

#
#   Control update client
#   Connects SUB socket to tcp://localhost:5556
#  
#
using ZMQ
using ProtoBuf
using LinearAlgebra
using ForwardDiff
using BlockDiagonals
using ControlSystems
using PyPlot
using SparseArrays
using OSQP

include("/home/fausto/protobuff/juliaout/filtered_state_msg_pb.jl")
include("/home/fausto/protobuff/juliaout/messaging.jl")

#connect to state publisher 

#contextsub = Context()
#socket1 = Socket(contextsub, SUB)

#println("Collecting state information...")
#connect(socket1, "tcp://localhost:5556") #change to correct one

#subscribe(socket1)

#message = recv(socket)
# message



#Connect to cpp script 
#contextpub = Context()
#socket2 = Socket(contextpub, PUB)
#bind(socket2, "tcp://*:5555")
#iob = IOBuffer()

#functions used in the quaternion math
function hat(v) #skew symmetric matrix operator
    return [0 -v[3] v[2];
            v[3] 0 -v[1];
            -v[2] v[1] 0]
end
function L(q) #the function takes a quaternion as input
    s = q[1] #scalar part of quaternion
    v = q[2:4] #vector part of quaternion
    L = [s    -v';
         v  s*I+hat(v)]
    return L
end

T = Diagonal([1; -ones(3)]) # to find the inverse of a quaternion
H = [zeros(1,3); I]  #zero the scalar part of quaternion
function qtoQ(q) #figure out what this function is for
    return H'*T*L(q)*T*L(q)*H
end

function G(q)
    G = L(q)*H
end

function rptoq(phi)# mapping between rodrigues parameter and quaternion
    (1/sqrt(1+phi'*phi))*[1; phi] #known as the Cayley map
end
function qtorp(q) #mapping between quaternion to rodrigues parameter
    q[2:4]/q[1]
end

# updated quadrotor parameters
m = 1.776 # in kg
l = 0.28 #in m
J = [0.01566089 0.00000318037 0; 0.00000318037 0.01562078 0; 0 0 0.02226868]
g = 9.81
kt=0.11
km=0.044

umin = [0;0;0;0]
umax = [0.3*m*g;0.3*m*g;0.3*m*g;0.3*m*g]

h = 0.05 #20 Hz

Nx = 13     #number of states (w quaternion)
Nx_ = 12    #number of states (linearized using rodrigues parameters)
Nu = 4     # number of controls
Tfinal = 5.0 #final time (may change if it is not discrete time)
Nt = Int(Tfinal/h)+1    # number of time steps
thist = Array(range(0,h*(Nt-1), step=h));


function E(q)
    E = BlockDiagonal([1.0*I(3), G(q), 1.0*I(6)])
end

function quad_dynamics(x,u)
    r = x[1:3]
    q = x[4:7]
    v = x[8:10]
    omega = x[11:13]
    Q = qtoQ(q)
    
    rdot = Q*v
    qdot = 0.5*L(q)*H*omega
    
    vdot = Q'*[0; 0; -g] + (1/m)*[zeros(2,4); kt*ones(1,4)]*u - hat(omega)*v
    
    omegadot = J\(-hat(omega)*J*omega + [0 l*kt 0 -l*kt; -l*kt 0 l*kt 0; km -km km -km]*u)
    
    return [rdot; qdot; vdot; omegadot]
end

function quad_dynamics_rk4(x,u)
    #RK4 integration with zero-order hold on u
    f1 = quad_dynamics(x, u)
    f2 = quad_dynamics(x + 0.5*h*f1, u)
    f3 = quad_dynamics(x + 0.5*h*f2, u)
    f4 = quad_dynamics(x + h*f3, u)
    xn = x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)
    xn[4:7] .= xn[4:7]/norm(xn[4:7]) #re-normalize quaternion to reduce error
    return xn #returns the state at a future timestep
end

#Initial Conditions
uhover = (m*g/4)*ones(4) #gravity compensation for a hover
r0 = [0.0; 0; 1.0] #1 meter off the ground
q0 = [1.0; 0; 0; 0]
v0 = zeros(3)
omega0 = zeros(3) #stable hover, no velocities
x0 = [r0; q0; v0; omega0]; #initial/reference state

#Linearize dynamics about hover
A = ForwardDiff.jacobian(x->quad_dynamics_rk4(x,uhover),x0)
B = ForwardDiff.jacobian(u->quad_dynamics_rk4(x0,u),uhover);

quad_dynamics_rk4(x0, uhover)

print("A matrix", "\n")
print(size(A), "\n")
print("B matrix", "\n")
print(size(B), "\n")

#reduced system to make it controllable 
A_ = Array(E(q0)'*A*E(q0));
B_ = Array(E(q0)'*B);

#cost weights
Q = Array(1.0*I(Nx_));
R = Array(0.1*I(Nu));
Qn = Array(1.0*I(Nx_));

#cost function
function cost(xhist,uhist)
	cost = 0.5*xhist[:,end]'*Qn*xhist[:,end]
	for k = 1:(size(xhist,2)-1)
		cost = cost + 0.5*xhist[:,k]'*Q*xhist[:,k] + 0.5*(uhist[k]'*R*uhist[k])[1]
	end
	return cost
end

P = dare(A_,B_,Q,R)

K = dlqr(A_,B_,Q,R)

#Feedback controller
#Build QP matrices for OSQP
Nh = 20
Nu = 4
U = kron(Diagonal(I,Nh), [I zeros(Nu,Nx_)]) #Matrix that picks out all u
Θ = kron(Diagonal(I,Nh), [0 0 0 0 1 0 0 0]) #Matrix that picks out all x3 (θ)
H = sparse([kron(Diagonal(I,Nh-1),[R zeros(Nu,Nx_); zeros(Nx_,Nu) Q]) zeros((Nx_+Nu)*(Nh-1), Nx_+Nu); zeros(Nx_+Nu,(Nx_+Nu)*(Nh-1)) [R zeros(Nu,Nx_); zeros(Nx_,Nu) P]])
b = zeros(Nh*(Nx_+Nu))
C = sparse([[B_ -I zeros(Nx_,(Nh-1)*(Nu+Nx_))]; zeros(Nx_*(Nh-1),Nu) [kron(Diagonal(I,Nh-1), [A_ B_]) zeros((Nh-1)*Nx_,Nx_)] + [zeros((Nh-1)*Nx_,Nx_) kron(Diagonal(I,Nh-1),[zeros(Nx_,Nu) Diagonal(-I,Nx_)])]])
#D = [C; U]
#lb = [zeros(Nx*Nh); kron(ones(Nh),umin-u_hover)]
#ub = [zeros(Nx*Nh); kron(ones(Nh),umax-u_hover)]
print(size(C))
print(size(U))
D = [C; U]
lb = [zeros(Nx_*Nh); kron(ones(Nh),umin-uhover); -0.3*ones(Nh)]
ub = [zeros(Nx_*Nh); kron(ones(Nh),umax-uhover); 0.3*ones(Nh)]

prob = OSQP.Model()
OSQP.setup!(prob; P=H, q=b, A=D, l=lb, u=ub, verbose=false, eps_abs=1e-8, eps_rel=1e-8, polish=1);



#MPC Controller
function mpc_controller(t,x,xref)
    
    #Update QP problem
    lb[1:6] .= -A_*x
    ub[1:6] .= -A_*x
    
    for j = 1:(Nh-1)
        b[(Nu+(j-1)*(Nx+Nu)).+(1:Nx)] .= -Q*xref
    end
    b[(Nu+(Nh-1)*(Nx+Nu)).+(1:Nx)] .= -P*xref
    
    OSQP.update!(prob, q=b, l=lb, u=ub)

    #Solve QP
    results = OSQP.solve!(prob)
    Δu = results.x[1:Nu]

    return uhover + Δu
end



function closed_loop(x0,controller,N)
    xhist = zeros(length(x0),N)
    u0 = controller(1,x0)
    uhist = zeros(length(u0),N-1)
    uhist[:,1] .= u0
    xhist[:,1] .= x0
    for k = 1:(N-1)
        uk = controller(k,xhist[:,k])
        uhist[:,k] = max.(min.(umax, uk), umin) #enforce control limits
        xhist[:,k+1] .= quad_dynamics_rk4(xhist[:,k],uhist[:,k])
    end
    return xhist, uhist
end

x_ref = [0.0; 1.0; 0; 0; 0; 0]
x0 = [1.0; 2.0; 0; 0; 0; 0]
xhist1, uhist1 = closed_loop(x0, (t,x)->mpc_controller(t,x,x_ref), Nt);






plot(thist,xhist[1,:], label="x LQR")
xlabel("time")
legend()
show()



