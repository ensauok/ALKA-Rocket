import numpy as np
import matplotlib.pyplot as plt

def three_dof_body_axes(Fx, Fz, My, u0=0.0, w0=0.0, theta0=0.0, q0=0.0, pos0=[0.0, 0.0], mass=0, inertia=0.0, g=9.81, dt=0.01, duration=10):
    pos = np.array(pos0, dtype=float) 
u = u0
w = w0
vel = np.array([u, w]) 
theta = theta0
q = q0
ax = Fx[0] / mass
az = Fz[0] / mass - g
theta_list = [theta]
q_list = [q]
dqdt_list = [0]
pos_list = [pos.copy()]
velocity_list = [vel.copy()]
acceleration_list = [np.array([ax, az])]
if pos[1] <= 0 and t > 2: 
    break
def three_dof_body_axes(Fx, Fz, My, u0=0.0, w0=0.0, theta0=0.0, q0=0.0, pos0=[0.0, 0.0], mass=0, inertia=0.0, g=9.81, dt=0.01, duration=10):
    pos = np.array(pos0, dtype=float)
    
 
    u = u0
    w = w0
    theta = theta0
    q = q0
    vel = np.array([u, w])
    
  
    ax = Fx[0] / mass
    az = Fz[0] / mass - g
    
    
    theta_list = [theta]
    q_list = [q]
    dqdt_list = [0]
    pos_list = [pos.copy()]
    velocity_list = [vel.copy()]
    acceleration_list = [np.array([ax, az])]

    for t in np.arange(dt, duration + dt, dt):  
       
        ax = Fx[int(t/dt)] / mass
        az = Fz[int(t/dt)] / mass - g
        
      
        dqdt = My[int(t/dt)] / inertia
        
    
        u += ax * dt
        w += az * dt
        q += dqdt * dt
        
      
        pos += vel * dt
        vel = np.array([u, w])
        
       
        theta += q * dt
        
    
        theta_list.append(theta)
        q_list.append(q)
        dqdt_list.append(dqdt)
        pos_list.append(pos.copy())
        velocity_list.append(vel.copy())
        acceleration_list.append(np.array([ax, az]))
        
        
        if pos[1] <= 0 and t > 2:  
            break
    
    return {
        'theta' : np.array(theta_list),
        'q' : np.array(q_list),
        'dqdt' : np.array(dqdt_list),
        'pos' : np.array(pos_list),
        'velocity' : np.array(velocity_list),
        'acceleration' : np.array(acceleration_list)
    }