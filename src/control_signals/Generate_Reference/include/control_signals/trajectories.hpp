
#ifndef __TRAJECTORIES__
#define __TRAJECTORIES__	

constexpr int TrajectoryLength = 49;

namespace WRF {
	 
     extern float Healthy_RH[];
     extern float Healthy_RK[];
     extern float Healthy_RA[];
     extern float Healthy_LH[];
     extern float Healthy_LK[];
     extern float Healthy_LA[];

     extern float User_Traj_RK[];
     extern float User_Traj_RA[];
     extern float User_Traj_LK[];
     extern float User_Traj_LA[];

     extern float QUAD_traj[];
     extern float HAMS_traj[];
     extern float TA_traj[];
     extern float CALF_traj[];
	 

}


#endif