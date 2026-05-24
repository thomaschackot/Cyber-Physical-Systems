using LinearAlgebra
using ControlSystems
using Plots

A = [
    1.0026   0.0050   0.0;
    1.0399   1.0026   0.0;
   -0.0675  -0.0002   1.0
]

B = [
   -0.0843;
   -33.7508;
    39.2131
]

# TASK 1 OPEN LOOP ANALYSIS
println("Eigen values of A: ",eigvals(A))

# TASK 2 CONTROLLABILITY
R = ctrb(A, B)
println("Reachability matrix: ", R)

println("Rank of R: ",rank(R))

# TASK 3 CONTROLLER CONSTRUCTION
poles = [0.7, 0.75, 0.8]
K = -place(A, B, poles)
println(K)

#TASK 4 CLOSED-LOOP VERIFICATION
A_cl = A + B*K
println(A_cl)

Acl_eigval = eigvals(A_cl)
print(Acl_eigval)

#5. SIMULATION
Ts = 0.005
Tfinal = 0.5
t = 0:Ts:Tfinal

initial_conditions = [
    [0.05, 0.0, 0.0],
    [0.10, 0.0, 0.0],
    [-0.08, 0.0, 0.0]
]

for (idx, x0) in enumerate(initial_conditions)

    x = zeros(3, length(t))
    u = zeros(length(t))

    x[:,1] = x0

    for k in 1:length(t)-1
        u[k] = (K * x[:,k])[1]
        x[:,k+1] = A*x[:,k] + B*u[k]
    end

    p1 = plot(t, x[1,:],
        xlabel="Time [s]",
        ylabel="θ",
        title="Pendulum Angle $(idx)",
        label="θ")

    p2 = plot(t, x[2,:],
        xlabel="Time [s]",
        ylabel="θdot",
        title="Pendulum Angular Velocity $(idx)",
        label="θdot")

    p3 = plot(t, x[3,:],
        xlabel="Time [s]",
        ylabel="φdot",
        title="Base Angular Velocity $(idx)",
        label="φdot")

    p4 = plot(t, u,
        xlabel="Time [s]",
        ylabel="u",
        title="Input Torque $(idx)",
        label="u")

    display(p1)
    display(p2)
    display(p3)
    display(p4)
end

