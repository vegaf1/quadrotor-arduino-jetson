# This node is run of the Jetson, it is a simple LQR controller to
# stabilize the quadrotor around a hover

module LqrHoverController
    using TOML
    using ZMQ
    using ProtoBuf
    using LinearAlgebra
    using ForwardDiff
    using BlockDiagonals
    using ControlSystems
    using PyPlot 

    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    using .PubSubBuilder

    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/motors_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")

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

#reduced system to make it controllable 
A_ = Array(E(q0)'*A*E(q0));
B_ = Array(E(q0)'*B);

#cost weights
Q = Array(1.0*I(Nx_));
R = Array(0.1*I(Nu));

K = dlqr(A_,B_,Q,R)

function quadcontroller(x)
	#change from quaternion to rod param
	q0 = x0[4:7]
	q = x[4:7]
	phi = qtorp(L(q0)'*q)

	deltax = [x[1:3]-r0; phi; x[8:10]-v0; x[11:13]-omega0]

	u = uhover - K*deltax
	
	return u

end




    function motor_commander(filtered_state_sub_ip::String, filtered_state_sub_port::String,
                             motor_pub_ip::String, motor_pub_port::String;
                             freq::Int64=200, debug::Bool=false)
        ctx = Context(1)

        # Initalize Subscriber threads
        state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                               quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                               vel_x=0., vel_y=0., vel_z=0.,
                               ang_x=0., ang_y=0., ang_z=0.)
        state_sub() = subscriber_thread(ctx, state, quad_info_sub_ip, quad_info_sub_port)

        # Setup and Schedule Subscriber Tasks
        state_thread = Task(state_sub)
        schedule(state_thread)

        # Setup Filtered state publisher
        motors = MOTORS(front_left=0., front_right=0., back_right=0., back_left=0.,
                        time=0.)
        quad_pub = create_pub(ctx, quad_info_pub_ip, quad_info_pub_port)
        iob = PipeBuffer()

        state_time = time()

        try
            while true
                # Prediction
                if state.time > state_time
                    # TODO: Run controller here
		    quadstate[1] = state.pos_x
		    quadstate[2] = state.pos_y
		    quadstate[3] = state.pos_z
		    quadstate[4] = state.quat_x
		    quadstate[5] = state.quat_y
		    quadstate[6] = state.quat_z
		    quadstate[7] = state.quat_w
		    quadstate[8] = state.vel_x
		    quadstate[9] = state.vel_y
		    quadstate[10] = state.vel_z
		    quadstate[11] = state.ang_x
		    quadstate[12] = state.ang_y
		    quadstate[13] = state.ang_z

                    usend = quadcontroller(quadstate)
		    
		    motors.front_left = usend[1]
		    motors.front_right = usend[2]
		    motors.back_right = usend[3]
		    motors.back_left = usend[4]

                    writeproto(iob, motors)
                    ZMQ.send(quad_pub, take!(iob))

                    state_time = state.time
                end
            end
        catch e
            close(ctx)
            if e isa InterruptException
                println("Process terminated by you")
            else
                rethrow(e)
            end
        end
    end

    # Launch IMU publisher
    function main()
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        zmq_filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        zmq_filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]
        zmq_motors_state_ip = setup_dict["zmq"]["jetson"]["motors"]["server"]
        zmq_motors_state_port = setup_dict["zmq"]["jetson"]["motors"]["port"]

        fs_pub() = motor_commander(zmq_filtered_state_ip, zmq_filtered_state_port,
                                   zmq_motors_state_ip, zmq_motors_state_port;
                                   freq=200, debug=false)
        fs_thread = Task(fs_pub)
        schedule(fs_thread)

        return fs_thread
    end
end
