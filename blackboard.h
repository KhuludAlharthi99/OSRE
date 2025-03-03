

#ifndef SWARM_SIMULATIONTEMP_H
#define SWARM_SIMULATIONTEMP_H

#include <array>
#include <vector>
#include <map>






struct blackboard {

    static const int Swarm_size=20; 
    static const int Swarm_size2=20;
    static const int Max_Swarm_size=20;

    static const int simulation_timesteps=600; 
    static inline double Swarm_speed=25; 
    static inline double Swarm_time_step=0.1;


    static inline int BT_size=4;

    static inline double High_density=3;
    //simulation
    static inline double robot_radius=12.5; 
    static inline double sensing_radius=22.5; 

    static inline double local_radius=robot_radius+sensing_radius;
    static inline double neighbours_in_radius=47.5;
    static inline int wait_time=simulation_timesteps/6;



    static inline double crossover_rate = 0.4; 
    static inline double crossover_rate2 = 0.35; 
    static inline double new_BT_rate2 = 0.4;
    static inline double mutation_rate = 0.15; 
    static inline int Elite_number = 3;
    static inline int tournament_size=3;
    static inline double local_search_rate=0.05;
    static inline int local_search_gen=15;
    static inline int local_search_gen2=30;
    static inline double pruning_rate = 0.5;
    static inline double random_immigration_rate = 0.2;


     struct env {
        bool objects_in_env=false;
        int objects_num;
         int swarm_size;
         vector<double> Initial_objects_X;
         vector<double> Initial_objects_Y;
         vector<float> Initial_swarm_X;
         vector<float> Initial_swarm_Y;
        vector<double> objects_X;
        vector<double> objects_Y;
        double objects_Width=14; //Simulation
        double objects_Width2=objects_Width/2;
        vector<vector<double>> objects_Data_X;
        vector<vector<double>> objects_Data_Y;
        vector<bool> objects_picked;
        bool areas_in_env=false;
        int areas_num;
        vector<double> areas_X;
        vector<double> areas_Y;
        vector<double> areas_Width;
        bool obstacles_in_env=false;
        int  obstacles_num;
        vector<double> obstacles_X;
        vector<double> obstacles_Y;
        vector<double> obstacles_X2;
        vector<double> obstacles_Y2;
    } ;


    float Distances [Max_Swarm_size][Max_Swarm_size]={0};
    float DistancesX [Max_Swarm_size][Max_Swarm_size]={0};
    float DistancesY [Max_Swarm_size][Max_Swarm_size]={0};

    int Msgs [Swarm_size]={0};
    int tick_times [Swarm_size]={0};
    double dispersion_x [Swarm_size]={0};
    double dispersion_y [Swarm_size]={0};
    double x_force =0.0;
    double y_force =0.0;
    double x_NEWS =0.0;
    double y_NEWS =0.0;
    double x_neighbour_force =0.0;
    double y_neighbour_force  =0.0;
    double x_heading =0.0;
    double y_heading =0.0;
    double separation_dist =2;
    double separation_dist2 =4;
    double separation_force =0;
    double separation_decay =0;
    double forces =0.0;
    float NEWS_forces=0;
    int move_backward=0;
    int attraction_force;
    int repulsion_force;
    int attraction_tick;
    int repulsion_tick;
    int boundary_force;
    int NE_force;
    int NW_force;
    int SE_force;
    int SW_force;

    vector<float> originalX;
    vector<float> originalY;

    vector<vector<float>> Final_Trajectories_x;
    vector<vector<float>> Final_Trajectories_y;



    struct Agent { double x; double y; double heading;  bool stop= false; int message1; int object_ID;
        bool picked_object=false; bool inside_area=false; int picked_object_t=0; int inside_area_t=0;};

    Agent agent;
    int agentID;

    struct Environment {
        bool areas = false;
        bool objects = false;
        bool obstacles = false;
        bool msgs = false;
    };



    struct Node
    {
        int node, child_num;
        float param;
    };


    std::map< int, int > Nodes_tick_times;
    std::map< int, int > Nodes_success_times;

    env BT_Env ;

    struct original
    {
        vector<vector<double>> original_metrics;
        double SwarmX [simulation_timesteps][Swarm_size];
        double SwarmY [simulation_timesteps][Swarm_size];
        vector<Node>  BTree;
        env BT_Env;
    };

    static original original_data;

    struct Environment_metrics
    {
        int picked_obj_num=0;
        int drop_obj_num=0;
        int inside_area_times=0;
        vector<float> picked_objs;
        vector<float> drop_objs;
        vector<float> in_area_times;

    };

    Environment_metrics Env_metrics;
    Agent agents[Swarm_size];


};




#endif //SWARM_SIMULATIONTEMP_H
