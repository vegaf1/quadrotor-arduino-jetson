#!/usr/bin/env julia

#
#   Control update client
#   Connects SUB socket to tcp://localhost:5556
#  
#

using ZMQ
import Pkg; Pkg.activate(@__DIR__); Pkg.instantiate()
using LinearAlgebra
using ForwardDiff
using BlockDiagonals
using ControlSystems

#functions used in the quaternion math
function hat(v) #skew symmetric matrix operator
	return [0 -v[3] v[2];
	        v[3] 0 -v[1];
		-v[2] v[1] 0]
end

function L(q) #the function takes a quaternion as input
	s = q[1] #scalar part of quaternion
	v = q[2:4] #vector part of quaternion
	L = [s -v';
	     v s*I+hat(v)]
	return L
end

T = Diagonal([1; -ones(3)]) # to find the inverse of a quaternion
H = [zeros(1,3); I] #zero the scalar part of quaternion

function qtoQ(q) #figure out what this function is for
	return H'*T*L(q)*T*L(q)*H
end

function G(q)
	G = L(q)*H
end

function rodtoquat(phi) # mapping between rodrigues parameter and quaternion
	(1/sqrt(1+phi'*phi))*[1;phi] #known as the Cayley map
end

function quattorod(q)
	q[2:4]/q[1] #mapping between quaternion to rodrigues parameter
end

#quadrotor parameters
m = 0.5
l = 0.1750
J = Diagonal([0.0023, 0.0023, 0.004])
g = 9.81
kt = 1.0
km = 0.0245

h = 0.05 # set at 20 hz

Nx = 13 #number of states (w quaternion)
Nx_ = 12 #number of states (linearized using rodrigues parameters)
Nu = 4 #number of controls
Tfinal = 5.0 #final time (may change if it is not discrete time)
Nt = Int(Tfinal/h)+1 #number of timesteps 
thist = Array(range(0,h*(Nt-1), step=h));

function E(q)
    E = BlockDiagonal([1.0*I(3), G(q), 1.0*I(6)])
end

#function for quadrotor dynamics
function quad_dynamics(x,u)
	r = x[1:3]
	q = x[4:7]
	v = x[8:10]
	omega = x[11:13]
	Q = qtoQ(q)

	rdot = Q*v
	qdot = 0.5*L(q)*H*omega
	vdot = Q'*[0; 0; -g] + (1/m)*[zeros(2,4); kt*ones(1,4)]*u-hat(omega)*v
	omegadot = J\(-hat(omega)*J*omega+[0 l*kt 0 -l*kt; -l*kt 0 l*kt 0; km -km km -km]*u)

	return(rdot; qdot; vdot; omegadot)

end

function quad_dynamics_rk4(x,u)
	#runga kutta method with zero order hold on u 
	f1 = quad_dynamics(x,u)
	f2 = quad_dynamics(x+0.5*h*f1, u)
	f3 = quad_dynamics(x+0.5*h*f2, u)
	f4 = quad_dynamics(x+h*f3, u)
	xn = x+(h/6.0)*(f1+2*f2+2*f3+f4)
	xn[4:7] = xn[4:7]/norm(xn[4:7]) #find out what this is for? maybe reduce error?
	return xn #returns the state at a future timestep
end

#Initial Conditions
uhover = (m*g/4)*ones(4)
r0 = [0.0; 0; 1.0] #1 meter off the ground
q0 = [1.0; 0; 0; 0] # no rotation
v0 = zeros(3)
omega0 = zeros(3) #stable hover
x0 = [r0; q0; v0; omega0]; #initial state

#Linearize the dynamics about the hover 

A = ForwardDiff.jacobian(x->quad_dynamics_rk4(x,uhover),x0)
B = ForwardDiff.jacobian(u->quad_dynamics_rk4(x0,u), uhover)

print(rank(A))
#print("A matrix")
#print(A)
#print("B Matrix")
#print(B)

#context = Context()
#socket = Socket(context, SUB)

#println("Collecting state information...")
#connect(socket, "tcp://localhost:5556") #change to correct one

# Subscribe to zipcode, default is NYC, 10001
#zip_filter = length(ARGS) > 0 ? int(ARGS[1]) : 10001

#subscribe(socket)

# Process 10 updates
#update_nbr = 10

#for update in 1:update_nbr
#    global total_temp
#    message = unsafe_string(recv(socket))
#end

#avg_temp = total_temp / update_nbr

#println("Average temperature for zipcode $zip_filter was $(avg_temp)F")

# Making a clean exit.
#close(socket)
#close(context)
