import numpy as np
import matplotlib.pyplot as plt


x=[]
y=[]
xgt=[]
ygt=[]
time=[]
fig, (ax1, ax2, ax3) = plt.subplots(3)
#myfile<< best_particle.x<< " "<< best_particle.y<< " " <<gt_x << " "<<gt_y<<" "<<RMSE(0)<<" "<< RMSE(1)<<" " <<duration.count()<<'\n';
with open('./src/particle/res.txt','r') as file:
    # reading each line    
    for line in file:
        # reading each word        
        word = line.split()
        x.append(float(word[0]))
        y.append(float(word[1]))  
        time.append(float(word[2]))

with open('pf_slam.txt','r') as file:
    # reading each line    
    for line in file:
        # reading each word        
        word = line.split()
        xgt.append(float(word[1]))
        ygt.append(float(word[2])) 

# Stessa scala per ax1 e ax2: x da -3 a 4, y auto-scalata ma sincronizzata
xlim = (-3, 4)
ylim = (min(min(y), min(ygt)) - 2, max(max(y), max(ygt)) + 2)  # ylims uguali basati su entrambi i dataset

ax1.scatter(x, y,color='blue', s=5)
ax1.set_xlim(xlim)
ax1.set_ylim(ylim)
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_title('Particle Filter Estimates')
ax1.grid(True, alpha=0.3)

ax2.scatter(xgt, ygt, color='green', s=5)
ax2.set_xlim(xlim)
ax2.set_ylim(ylim)
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_title('Ground Truth Trajectory')
ax2.grid(True, alpha=0.3)

t = [i for i in range(len(time))]
ax3.plot(t, time, color='red')
ax3.set_xlabel('Time step')
ax3.set_ylabel('Time (s)')
ax3.set_title('Computation Time')
ax3.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()
