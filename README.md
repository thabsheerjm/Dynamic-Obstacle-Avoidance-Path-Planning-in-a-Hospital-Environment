# RBE550: Motion Planing - Final Project
**Title :** Dynamic Obstacle Avoidance & Path Planning in a Hospital Environment

## Description
This project aims to address the challenge of safe and efficient mobility for mobile robots in hospital environments. Prompt delivery of supplies, equipment, and medication is critical in hospitals, but the environment is dynamic and complex, with various obstacles such as equipment, medical staff, and patients. The conventional path planning techniques such as Rapidly-exploring Random Trees (RRT) or Probabilistic Roadmap (PRM) make assumptions about static impediments and cannot manage non-holonomic restrictions. Therefore, the project aims to implement path planning algorithms that are efficient in dynamic environments and can take into account the non-holonomic limits of mobile robots. By doing so, the project aims to improve the efficiency and safety of mobile robots in hospital environments, ultimately improving the quality of patient care.
For this project, we have tested out out three different planning algorithms, namely D*, Hybrid Potential Based PRM (HPPRM), Spline based Dynamic Windows Approach(DWA).

## Requirements
* Python3.8 or above
* Ros1
* Unity 2020.3

## How to Run
### D*
* 2-D implementation:  
 In the d2_imp folder:  
  
The D* toy problem is run in main.py of the 2d_imp folder   
	parameters:  
		scale - how much the map is scaled from the unity resolution of 0.05m/grid square  
		cspace - true to pull the map from move_base/global_costmap, false to get the /map csv's (scale 3 and up lose doorways with cspace as true)  
		show_static_map - visualization bool for the starting orientation  
		is_dynamic - set as True to use the dynamic maps (only works for scale 10 as I had to manually alter the csvs)  
  
To run adjust the parameters and run main.py. The maps need to stay in the same path location (in the ros node PathPlanning) or alter the file paths in lines 40-44 to reflect the correct path.   
  
To make more maps you can run generate_maps.py  
The map generator requires unity and ROS to be running becasue it pulls the maps from the ROS messages  
roslaunch gopher_unity_endpoint gopher_presence.launch  
rosrun path_planning generate_maps.py  
  
	parameters:  
		map_scale - how much the map is scaled from the hospitals 0.05m/grid  
		with_cspace - adjust which message the node recieves the map from, /map or /move_base/global_costmap  
			  
* D* Unity Implementation (little broken):  
The files are in the d_star folder  
It can be run by using the following commands  
  
roslaunch gopher_unity_endpoint gopher_presence.launch  
rosrun path_planning d_star.py  
  
In the ROS setup move_base has an altered topic to publish messages to so using hte pose in RVIZ controls the robto using d* and not move_base   
  
  
### HPPRM


### Spline based DWA

 * Navigate to the directory /path_planning/src/dwa_spline/
 * Run the python file DWA_main.py:  
 '''
   $ python3 DWA_main.py'
 '''
   

