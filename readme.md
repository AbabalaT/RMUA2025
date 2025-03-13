## RMUA2025
# Included repository
1. ros-robot-localization  (not use now)  
2. Ego-planner V2
3. Fast-Lio (planned to be replaced)

# TODO
- Optimize trajectory of some roads, make sure the feasibility
- weak power detection
- check drag force calculation (seemed not right)

# remote controll  
1. roll  
2. pitch  
3. throttle  
4. yaw  

# Mode  
- off-board (-1)  
- disable (0)  
- position (1)  
- stable (2)  
- acro (3)  

default: -1  
the rc node will set mode param and advertise float32 topic