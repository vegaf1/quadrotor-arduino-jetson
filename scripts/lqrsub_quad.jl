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

#umin = [0;0;0;0]
#umax = [0.3*m*g;0.3*m*g;0.3*m*g;0.3*m*g]

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
#Qn = Array(1.0*I(Nx_));

K = dlqr(A_,B_,Q,R)

#Feedback controller
function quadcontroller(x)
	#change from quaternion to rod param
	q0 = x0[4:7]
	q = x[4:7]
	phi = qtorp(L(q0)'*q)

	deltax = [x[1:3]-r0; phi; x[8:10]-v0; x[11:13]-omega0]

	u = uhover - K*deltax
	
	return u

end

#Simulate quad
uhist = zeros(Nu,Nt)
xhist = zeros(Nx,Nt)
xhist[:,1] = [r0+randn(3); L(q0)*rptoq(0.7*randn(3)); v0; omega0]
for k = 1:(Nt-1)
    uhist[:,k] = quadcontroller(xhist[:,k])
    xhist[:,k+1] = quad_dynamics_rk4(xhist[:,k],uhist[:,k])
    print(uhist[:,k])
end

plot(thist,xhist[1,:], label="x LQR")
xlabel("time")
legend()
show()




#print(uhist)
#

#cost function. Quadratic
#function Jcost(xhist,uhist)
#	cost=0.5*xhist[:,end]'*Qn*xhist[:,end]
#	for k= 1:(N-1)
#		cost = cost+0.5*xhist[:,k]'*Q*xhist[:,k]+0.5*(uhist[k]'*r*uhist[k])[1]
#	end
#	return cost
#end

#P = zeros(Nx_,Nx_,Nt)
#K = zeros(Nu, Nx_, Nt-1)

#P[:,:,Nt] .= Qn

#Backward Riccati recursion
#for k = (Nt-1):-1:1
#    K[:,:,k] .= (R + B_'*P[:,:,k+1]*B_)\(B_'*P[:,:,k+1]*A_)
#    P[:,:,k] .= Q + A_'*P[:,:,k+1]*(A_-B_*K[:,:,k])
#end


#forward rollout starting at x0
#xhist = zeros(Nx, Nt)
#xhist[:,1] = x0
#uhist = zeros(Nu, Nt-1)

#for k=1:(Nt-1)
#	uhist[:,k] .= -K[:,:,k] *xhist[:,k] the control inputs 
#	xhist[:,k+1] .= A*xhist[:,k] + B*uhist[k] the position based off of the dynamics
#end


# Plot x1 vs. x2, u vs. t, x vs. t, etc.
#times = range(0,h*(Nt-1), step=h)
#plot(times,xhist[1,:], label="positionx")
#plot(times,xhist[2,:], label="positiony")
#plot(times,xhist[3,:], label="positionz")
#xlabel("time")
#legend()

