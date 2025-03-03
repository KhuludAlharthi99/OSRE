
#include "blackboard.h"
#include <cstring>
#include <ctime>
#include <vector>
#include "BehaviorTree.h"
#include <GL\gl.h>
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
//#include <SDL.h>
//#include <SDL_opengl.h>
//#include <SDL_timer.h>
#include <time.h>
//#include <SDL.h>
#include <fstream>
#include <string>
#include <memory>



#ifndef SWARM_SWARM_H
#define SWARM_SWARM_H
typedef uint32_t u32;
#define WinWidth 500
#define WinHeight 500



class Swarm {

public:

    vector<blackboard::Node> BT1;
    BehaviorTree controller;
    //std::vector<blackboard::Agent> agent;
    blackboard::Agent agent[blackboard::Swarm_size];
    vector<vector<blackboard::Agent>> Swarm_History;
    vector<blackboard::Agent> Swarm_History2;
    vector<float> picked_objs;
    vector<float> drop_objs;
    vector<float> in_area_times;
    int max =250, min=-250;
    vector<vector<double>> objects_Data_X;
    vector<vector<double>> objects_Data_Y;

   std::unique_ptr<double[]> SwarmX = std::make_unique<double[]>(blackboard::simulation_timesteps*blackboard::Swarm_size);
   std::unique_ptr<double[]> SwarmY = std::make_unique<double[]>(blackboard::simulation_timesteps*blackboard::Swarm_size);



    vector<vector<vector<double>>> AllDistances;
    vector<double> dist;
    vector<vector<double>> Distances;

    void reset ()
    {
        Swarm_History.clear();
        Swarm_History2.clear();
        picked_objs.clear();
        drop_objs.clear();
        in_area_times.clear();
        AllDistances.clear();
        dist.clear();
        Distances.clear();

    }

    void reset (int swarm_size)
    {
        Swarm_History.clear();
        Swarm_History2.clear();
        picked_objs.clear();
        drop_objs.clear();
        in_area_times.clear();
        AllDistances.clear();
        dist.clear();
        Distances.clear();
       /* for(int i = 0; i < swarm_size; i++)
        {
            agent.push_back(blackboard::Agent());
        }*/
    }


    void set_BT (vector<blackboard::Node> BT1)
    {

        controller.BT.clear();
        controller.BT=BT1;
       // controller.BT = vector<blackboard::Node>(BT1);
        controller.BT.resize(BT1.size());

        std::fill( std::begin( controller.blackboard.Msgs ), std::end( controller.blackboard.Msgs ), 0 );
        std::fill( std::begin( controller.blackboard.tick_times ), std::end( controller.blackboard.tick_times ), 0 );
        std::fill( std::begin( controller.blackboard.dispersion_x ), std::end( controller.blackboard.dispersion_x ), 0 );
        std::fill( std::begin( controller.blackboard.dispersion_y ), std::end( controller.blackboard.dispersion_y ), 0 );

        controller.reset_env();
        controller.blackboard.Env_metrics.drop_obj_num=0;
        controller.blackboard.Env_metrics.picked_obj_num=0;
        controller.blackboard.Env_metrics.inside_area_times=0;

        controller.blackboard.Env_metrics.drop_objs.clear();
        controller.blackboard.Env_metrics.picked_objs.clear();
        controller.blackboard.Env_metrics.in_area_times.clear();
        objects_Data_X.clear();
        objects_Data_Y.clear();

        controller.generate_env();
        controller.blackboard.BT_Env=controller.BT_environment;

    }

    void set_BT (vector<blackboard::Node> BT1, blackboard::env BT_environment)
    {

        controller.BT.clear();
        controller.BT=BT1;
        // controller.BT = vector<blackboard::Node>(BT1);
        controller.BT.resize(BT1.size());

        std::fill( std::begin( controller.blackboard.Msgs ), std::end( controller.blackboard.Msgs ), 0 );
        std::fill( std::begin( controller.blackboard.tick_times ), std::end( controller.blackboard.tick_times ), 0 );
        std::fill( std::begin( controller.blackboard.dispersion_x ), std::end( controller.blackboard.dispersion_x ), 0 );
        std::fill( std::begin( controller.blackboard.dispersion_y ), std::end( controller.blackboard.dispersion_y ), 0 );

        controller.reset_env();
        controller.blackboard.Env_metrics.drop_obj_num=0;
        controller.blackboard.Env_metrics.picked_obj_num=0;
        controller.blackboard.Env_metrics.inside_area_times=0;

        controller.blackboard.Env_metrics.drop_objs.clear();
        controller.blackboard.Env_metrics.picked_objs.clear();
        controller.blackboard.Env_metrics.in_area_times.clear();

        objects_Data_X.clear();
        objects_Data_Y.clear();
        //controller.generate_env();
        controller.blackboard.BT_Env=BT_environment;

    }

    void set_BT_draw_env (vector<blackboard::Node> BT1)
    {

        controller.BT.clear();
        controller.BT=BT1;
        // controller.BT = vector<blackboard::Node>(BT1);
        controller.BT.resize(BT1.size());

        std::fill( std::begin( controller.blackboard.Msgs ), std::end( controller.blackboard.Msgs ), 0 );
        std::fill( std::begin( controller.blackboard.tick_times ), std::end( controller.blackboard.tick_times ), 0 );
        std::fill( std::begin( controller.blackboard.dispersion_x ), std::end( controller.blackboard.dispersion_x ), 0 );
        std::fill( std::begin( controller.blackboard.dispersion_y ), std::end( controller.blackboard.dispersion_y ), 0 );

        controller.reset_env();
        controller.blackboard.Env_metrics.drop_obj_num=0;
        controller.blackboard.Env_metrics.picked_obj_num=0;
        controller.blackboard.Env_metrics.inside_area_times=0;

        controller.blackboard.Env_metrics.drop_objs.clear();
        controller.blackboard.Env_metrics.picked_objs.clear();
        controller.blackboard.Env_metrics.in_area_times.clear();
        objects_Data_X.clear();
        objects_Data_Y.clear();

        controller.draw_env();
        controller.blackboard.BT_Env=controller.BT_environment;

    }

    void set_BT_draw_all_env (vector<blackboard::Node> BT1)
    {

        controller.BT.clear();
        controller.BT=BT1;
        // controller.BT = vector<blackboard::Node>(BT1);
        controller.BT.resize(BT1.size());

        std::fill( std::begin( controller.blackboard.Msgs ), std::end( controller.blackboard.Msgs ), 0 );
        std::fill( std::begin( controller.blackboard.tick_times ), std::end( controller.blackboard.tick_times ), 0 );
        std::fill( std::begin( controller.blackboard.dispersion_x ), std::end( controller.blackboard.dispersion_x ), 0 );
        std::fill( std::begin( controller.blackboard.dispersion_y ), std::end( controller.blackboard.dispersion_y ), 0 );

        controller.reset_env();
        controller.blackboard.Env_metrics.drop_obj_num=0;
        controller.blackboard.Env_metrics.picked_obj_num=0;
        controller.blackboard.Env_metrics.inside_area_times=0;

        controller.blackboard.Env_metrics.drop_objs.clear();
        controller.blackboard.Env_metrics.picked_objs.clear();
        controller.blackboard.Env_metrics.in_area_times.clear();
        objects_Data_X.clear();
        objects_Data_Y.clear();

        controller.draw_all_env();

        controller.blackboard.BT_Env=controller.BT_environment;

    }

    void set_BT_dread_vid_env (vector<blackboard::Node> BT1)
    {

        controller.BT.clear();
        controller.BT=BT1;
        // controller.BT = vector<blackboard::Node>(BT1);
        controller.BT.resize(BT1.size());

        std::fill( std::begin( controller.blackboard.Msgs ), std::end( controller.blackboard.Msgs ), 0 );
        std::fill( std::begin( controller.blackboard.tick_times ), std::end( controller.blackboard.tick_times ), 0 );
        std::fill( std::begin( controller.blackboard.dispersion_x ), std::end( controller.blackboard.dispersion_x ), 0 );
        std::fill( std::begin( controller.blackboard.dispersion_y ), std::end( controller.blackboard.dispersion_y ), 0 );

        controller.reset_env();
        controller.blackboard.Env_metrics.drop_obj_num=0;
        controller.blackboard.Env_metrics.picked_obj_num=0;
        controller.blackboard.Env_metrics.inside_area_times=0;

        controller.blackboard.Env_metrics.drop_objs.clear();
        controller.blackboard.Env_metrics.picked_objs.clear();
        controller.blackboard.Env_metrics.in_area_times.clear();
        objects_Data_X.clear();
        objects_Data_Y.clear();

        controller.read_video_env();

        controller.blackboard.BT_Env=controller.BT_environment;

    }

    void object_trajectories ()
    {
        for(int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            objects_Data_X.push_back(controller.blackboard.BT_Env.objects_X);
            objects_Data_Y.push_back(controller.blackboard.BT_Env.objects_Y);
        }
    }

    void set_BT (vector<blackboard::Node> BT1, blackboard::env BT_environment,bool area, bool objects, bool obstacles)
    {

        controller.BT.clear();
        controller.BT=BT1;
        // controller.BT = vector<blackboard::Node>(BT1);
        controller.BT.resize(BT1.size());

        std::fill( std::begin( controller.blackboard.Msgs ), std::end( controller.blackboard.Msgs ), 0 );
        std::fill( std::begin( controller.blackboard.tick_times ), std::end( controller.blackboard.tick_times ), 0 );
        std::fill( std::begin( controller.blackboard.dispersion_x ), std::end( controller.blackboard.dispersion_x ), 0 );
        std::fill( std::begin( controller.blackboard.dispersion_y ), std::end( controller.blackboard.dispersion_y ), 0 );

        controller.reset_env();
        controller.blackboard.Env_metrics.drop_obj_num=0;
        controller.blackboard.Env_metrics.picked_obj_num=0;
        controller.blackboard.Env_metrics.inside_area_times=0;

        controller.blackboard.Env_metrics.drop_objs.clear();
        controller.blackboard.Env_metrics.picked_objs.clear();
        controller.blackboard.Env_metrics.in_area_times.clear();

        objects_Data_X.clear();
        objects_Data_Y.clear();
        //controller.generate_env();
        controller.BT_environment=BT_environment;
        if (area==false)
        {
            controller.generate_env_areas();
        }
        if (objects==false)
        {
            controller.generate_env_objects();
        }
        if (obstacles==false)
        {
            controller.generate_env_obstacles();
        }
        controller.blackboard.BT_Env=controller.BT_environment;

    }

    void set_BT (vector<blackboard::Node> BT1, const std::vector<std::vector<double>>& objects_X, const std::vector<std::vector<double>>& objects_Y, int obj_num)
    {

        controller.BT.clear();
        controller.BT=BT1;
        // controller.BT = vector<blackboard::Node>(BT1);
        controller.BT.resize(BT1.size());

        std::fill( std::begin( controller.blackboard.Msgs ), std::end( controller.blackboard.Msgs ), 0 );
        std::fill( std::begin( controller.blackboard.tick_times ), std::end( controller.blackboard.tick_times ), 0 );
        std::fill( std::begin( controller.blackboard.dispersion_x ), std::end( controller.blackboard.dispersion_x ), 0 );
        std::fill( std::begin( controller.blackboard.dispersion_y ), std::end( controller.blackboard.dispersion_y ), 0 );

        controller.reset_env();
        controller.blackboard.Env_metrics.drop_obj_num=0;
        controller.blackboard.Env_metrics.picked_obj_num=0;
        controller.blackboard.Env_metrics.inside_area_times=0;

        controller.blackboard.Env_metrics.drop_objs.clear();
        controller.blackboard.Env_metrics.picked_objs.clear();
        controller.blackboard.Env_metrics.in_area_times.clear();

        objects_Data_X.clear();
        objects_Data_Y.clear();

        //controller.read_env2(objects_X,objects_Y,obj_num); // reading from simulation demos
        controller.read_env(objects_X,objects_Y,obj_num); // reading from real robot demo
        objects_Data_X=controller.BT_environment.objects_Data_X;
        objects_Data_Y=controller.BT_environment.objects_Data_Y;
        controller.blackboard.BT_Env=controller.BT_environment;

    }





    void create_BT_env (int depth)
    {
        //controller.Env_settings = env;
        //controller.BT_nodes_set_env();
        vector<int> actions;
        actions= {2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,19,20,21,22,23,24};
        controller.BT_nodes_set_from_list(actions);
        controller.nActions_RandomBT_generator(depth);
        controller.reset_env();
        controller.generate_env();
        controller.blackboard.BT_Env=controller.BT_environment;
    }

    void clear_distances ()
    {
        for(int i = 0; i < blackboard::Swarm_size; i++)
        {
            dist.push_back(0.0);
        }
        for(int i = 0; i < blackboard::Swarm_size; i++)
        {
            Distances.push_back(dist);
        }

    }

    void clear_distances (int swarm_size)
    {
        for(int i = 0; i < swarm_size; i++)
        {
            dist.push_back(0.0);
        }
        for(int i = 0; i < swarm_size; i++)
        {
            Distances.push_back(dist);
        }

    }

    void generate_swarm( )
    {
        clear_distances ();
        controller.blackboard.originalX.clear();
        controller.blackboard.originalY.clear();
        for (int i=0;i< blackboard::Swarm_size;i++)
        {
           // agent[i].x=((double) rand() / (RAND_MAX)) * 2 - 1;
           // agent[i].y=((double) rand() / (RAND_MAX)) * 2 - 1;
            agent[i].x=rand()%(max-min+1)+min;
            agent[i].y=rand()%(max-min+1)+min;
            agent[i].heading=0.0314*((-100) + (rand() % static_cast<int>(100 - (-100) + 1)));
            agent[i].picked_object=false;
            agent[i].object_ID=-1;
            SwarmX[i]=agent[i].x;
            SwarmY[i]=agent[i].y;
            controller.blackboard.originalX.push_back(agent[i].x);
            controller.blackboard.originalY.push_back(agent[i].y);
        }
        controller.blackboard.Nodes_tick_times.clear();
        controller.blackboard.Nodes_success_times.clear();
        for(int i=0; i<controller.BT.size();i++)
        {


            controller.blackboard.Nodes_tick_times[i]=0;
            controller.blackboard.Nodes_success_times[i]=0;

        }
        controller.local_index=((controller.BT.size()-1));
        controller.decode_BT4((controller.BT.size()-1));
    }

    void generate_swarm(int swarm_size)
    {
        clear_distances (swarm_size);
        controller.blackboard.originalX.clear();
        controller.blackboard.originalY.clear();
        for (int i=0;i< swarm_size;i++)
        {
            // agent[i].x=((double) rand() / (RAND_MAX)) * 2 - 1;
            // agent[i].y=((double) rand() / (RAND_MAX)) * 2 - 1;
            agent[i].x=rand()%(max-min+1)+min;
            agent[i].y=rand()%(max-min+1)+min;
            agent[i].heading=0.0314*((-100) + (rand() % static_cast<int>(100 - (-100) + 1)));
            agent[i].picked_object=false;
            agent[i].object_ID=-1;
            SwarmX[i]=agent[i].x;
            SwarmY[i]=agent[i].y;
            controller.blackboard.originalX.push_back(agent[i].x);
            controller.blackboard.originalY.push_back(agent[i].y);
        }
        controller.blackboard.Nodes_tick_times.clear();
        controller.blackboard.Nodes_success_times.clear();
        for(int i=0; i<controller.BT.size();i++)
        {


            controller.blackboard.Nodes_tick_times[i]=0;
            controller.blackboard.Nodes_success_times[i]=0;

        }
        controller.local_index=((controller.BT.size()-1));
        controller.decode_BT4((controller.BT.size()-1));
    }

    void initialize_swarm ()
    {
        clear_distances ();
        for (int i=0;i< blackboard::Swarm_size;i++)
        {
            // agent[i].x=((double) rand() / (RAND_MAX)) * 2 - 1;
            // agent[i].y=((double) rand() / (RAND_MAX)) * 2 - 1;
            agent[i].x=controller.blackboard.originalX[i];
            agent[i].y=controller.blackboard.originalY[i];
            agent[i].heading=0.0314*((-100) + (rand() % static_cast<int>(100 - (-100) + 1)));
            agent[i].picked_object=false;
            agent[i].object_ID=-1;
            SwarmX[i]=agent[i].x;
            SwarmY[i]=agent[i].y;
        }
        controller.blackboard.Nodes_tick_times.clear();
        controller.blackboard.Nodes_success_times.clear();
        for(int i=0; i<controller.BT.size();i++)
        {


            controller.blackboard.Nodes_tick_times[i]=0;
            controller.blackboard.Nodes_success_times[i]=0;

        }
        controller.local_index=((controller.BT.size()-1));
        controller.decode_BT4((controller.BT.size()-1));
    }

 

    void initialize_swarm (int swarm_size)
    {
        clear_distances (swarm_size);
        for (int i=0;i< swarm_size;i++)
        {
            // agent[i].x=((double) rand() / (RAND_MAX)) * 2 - 1;
            // agent[i].y=((double) rand() / (RAND_MAX)) * 2 - 1;
            agent[i].x=controller.blackboard.originalX[i];
            agent[i].y=controller.blackboard.originalY[i];
            agent[i].heading=0.0314*((-100) + (rand() % static_cast<int>(100 - (-100) + 1)));
            agent[i].picked_object=false;
            agent[i].object_ID=-1;
            SwarmX[i]=agent[i].x;
            SwarmY[i]=agent[i].y;
        }
        controller.blackboard.Nodes_tick_times.clear();
        controller.blackboard.Nodes_success_times.clear();
        for(int i=0; i<controller.BT.size();i++)
        {


            controller.blackboard.Nodes_tick_times[i]=0;
            controller.blackboard.Nodes_success_times[i]=0;

        }
        controller.local_index=((controller.BT.size()-1));
        controller.decode_BT4((controller.BT.size()-1));
    }

    void initialize_swarm2 ()
    {
        clear_distances ();
        for (int i=0;i< blackboard::Swarm_size;i+=1)
        {
            // agent[i].x=((double) rand() / (RAND_MAX)) * 2 - 1;
            // agent[i].y=((double) rand() / (RAND_MAX)) * 2 - 1;
            agent[i].x=controller.blackboard.originalX[i];
            agent[i].y=controller.blackboard.originalY[i];
            agent[i].heading=0.0314*((-100) + (rand() % static_cast<int>(100 - (-100) + 1)));
            agent[i].picked_object=false;
            agent[i].object_ID=-1;
            SwarmX[i]=agent[i].x;
            SwarmY[i]=agent[i].y;
        }
        for (int i=0;i< 2;i+=1)
        {
            int agent_id = rand()% blackboard::Swarm_size;
            // agent[i].x=((double) rand() / (RAND_MAX)) * 2 - 1;
            // agent[i].y=((double) rand() / (RAND_MAX)) * 2 - 1;
            agent[agent_id].x=rand()%(max-min+1)+min;
            agent[agent_id].y=rand()%(max-min+1)+min;
            agent[agent_id].heading=0.0314*((-100) + (rand() % static_cast<int>(100 - (-100) + 1)));
            agent[agent_id].picked_object=false;
            SwarmX[agent_id]=agent[agent_id].x;
            SwarmY[agent_id]=agent[agent_id].y;
        }
        controller.blackboard.Nodes_tick_times.clear();
        controller.blackboard.Nodes_success_times.clear();
        for(int i=0; i<controller.BT.size();i++)
        {


            controller.blackboard.Nodes_tick_times[i]=0;
            controller.blackboard.Nodes_success_times[i]=0;

        }
        controller.local_index=((controller.BT.size()-1));
        controller.decode_BT4((controller.BT.size()-1));
    }

    void initialize_swarm2 (int swarm_size)
    {
        clear_distances (swarm_size);
        for (int i=0;i< swarm_size;i+=1)
        {
            // agent[i].x=((double) rand() / (RAND_MAX)) * 2 - 1;
            // agent[i].y=((double) rand() / (RAND_MAX)) * 2 - 1;
            agent[i].x=controller.blackboard.originalX[i];
            agent[i].y=controller.blackboard.originalY[i];
            agent[i].heading=0.0314*((-100) + (rand() % static_cast<int>(100 - (-100) + 1)));
            agent[i].picked_object=false;
            agent[i].object_ID=-1;
            SwarmX[i]=agent[i].x;
            SwarmY[i]=agent[i].y;
        }
        for (int i=0;i< 5;i++)
        {
            int agent_id = rand()% swarm_size;
            // agent[i].x=((double) rand() / (RAND_MAX)) * 2 - 1;
            // agent[i].y=((double) rand() / (RAND_MAX)) * 2 - 1;
            agent[agent_id].x=rand()%(max-min+1)+min;
            agent[agent_id].y=rand()%(max-min+1)+min;
            agent[agent_id].heading=0.0314*((-100) + (rand() % static_cast<int>(100 - (-100) + 1)));
            agent[agent_id].picked_object=false;
            SwarmX[agent_id]=agent[agent_id].x;
            SwarmY[agent_id]=agent[agent_id].y;
        }
        controller.blackboard.Nodes_tick_times.clear();
        controller.blackboard.Nodes_success_times.clear();
        for(int i=0; i<controller.BT.size();i++)
        {


            controller.blackboard.Nodes_tick_times[i]=0;
            controller.blackboard.Nodes_success_times[i]=0;

        }
        controller.local_index=((controller.BT.size()-1));
        controller.decode_BT4((controller.BT.size()-1));
    }

    void initialize_swarm3 ()
    {
        clear_distances ();
        for (int i=0;i< blackboard::Swarm_size2;i+=1)
        {
            // agent[i].x=((double) rand() / (RAND_MAX)) * 2 - 1;
            // agent[i].y=((double) rand() / (RAND_MAX)) * 2 - 1;
            agent[i].x=controller.blackboard.originalX[i];
            agent[i].y=controller.blackboard.originalY[i];
            agent[i].heading=0.0314*((-100) + (rand() % static_cast<int>(100 - (-100) + 1)));
            agent[i].picked_object=false;
            agent[i].object_ID=-1;
            SwarmX[i]=agent[i].x;
            SwarmY[i]=agent[i].y;
        }
        for (int i=blackboard::Swarm_size2;i< blackboard::Swarm_size;i+=1)
        {

            agent[i].x=rand()%(max-min+1)+min;
            agent[i].y=rand()%(max-min+1)+min;
            agent[i].heading=0.0314*((-100) + (rand() % static_cast<int>(100 - (-100) + 1)));
            agent[i].picked_object=false;
            SwarmX[i]=agent[i].x;
            SwarmY[i]=agent[i].y;
        }
        controller.blackboard.Nodes_tick_times.clear();
        controller.blackboard.Nodes_success_times.clear();
        for(int i=0; i<controller.BT.size();i++)
        {


            controller.blackboard.Nodes_tick_times[i]=0;
            controller.blackboard.Nodes_success_times[i]=0;

        }
        controller.local_index=((controller.BT.size()-1));
        controller.decode_BT4((controller.BT.size()-1));
    }

    void compute_swarm_distance( ) {

        for (int i = 0; i < blackboard::Swarm_size; i++) {
            for (int j = 0; j < blackboard::Swarm_size; j++) {

                Distances[i][j] = sqrt(pow(agent[i].x - agent[j].x, 2) + pow(agent[i].y - agent[j].y, 2));
                controller.blackboard.Distances[i][j] = Distances[i][j];
                controller.blackboard.DistancesX[i][j] = agent[i].x - agent[j].x;
                controller.blackboard.DistancesY[i][j] = agent[i].y - agent[j].y;
            }
        }

        AllDistances.push_back(Distances);


    }

    void compute_swarm_distance(int swarm_size) {

        for (int i = 0; i < swarm_size; i++) {
            for (int j = 0; j < swarm_size; j++) {

                Distances[i][j] = sqrt(pow(agent[i].x - agent[j].x, 2) + pow(agent[i].y - agent[j].y, 2));
                controller.blackboard.Distances[i][j] = Distances[i][j];
                controller.blackboard.DistancesX[i][j] = agent[i].x - agent[j].x;
                controller.blackboard.DistancesY[i][j] = agent[i].y - agent[j].y;

            }
        }

        AllDistances.push_back(Distances);


    }

    void simulate_swarm()
    {

        double delta_x = 0.0;
        double delta_y = 0.0;
        double heading =0.0;

        compute_swarm_distance();
        std::copy(std::begin(agent), std::end(agent), std::begin(controller.blackboard.agents));


        //memcpy(AllDistances[0], controller.blackboard.Distances, sizeof(AllDistances[0]));
        //AllDistances.clear();

        if(controller.blackboard.BT_Env.objects_in_env)
        {
            controller.blackboard.BT_Env.objects_X=controller.blackboard.BT_Env.Initial_objects_X;
            controller.blackboard.BT_Env.objects_Y=controller.blackboard.BT_Env.Initial_objects_Y;
            objects_Data_X.push_back(controller.blackboard.BT_Env.objects_X);
            objects_Data_Y.push_back(controller.blackboard.BT_Env.objects_Y);

        }
        controller.blackboard.Final_Trajectories_x.push_back(controller.blackboard.originalX);
        controller.blackboard.Final_Trajectories_y.push_back(controller.blackboard.originalY);

        for(int i = 1; i < blackboard::simulation_timesteps; i++)
        {
            vector<float> x;
            vector <float> y;
            controller.blackboard.boundary_force=0;
            for (int j=0; j<blackboard::Swarm_size;j++)
            {
                controller.blackboard.tick_times[j] += 1;
                //Node::agent =agent[i];
                //Node::agentID=i;
                controller.blackboard.agent=agent[j];
                controller.blackboard.agentID=j;

                controller.blackboard.x_force=0.0;
                controller.blackboard.y_force=0.0;
                controller.blackboard.x_NEWS=0.0;
                controller.blackboard.y_NEWS=0.0;
                controller.blackboard.x_neighbour_force=0.0;
                controller.blackboard.y_neighbour_force=0.0;
                controller.blackboard.forces=0;
                controller.blackboard.NEWS_forces=0;
                controller.blackboard.separation_dist=8;
                controller.blackboard.separation_force=5;
                controller.blackboard.separation_decay=0.01;
                controller.blackboard.move_backward=0;
                controller.blackboard.attraction_force=0;
                controller.blackboard.repulsion_force=0;
                controller.blackboard.NE_force=0;
                controller.blackboard.NW_force=0;
                controller.blackboard.SE_force=0;
                controller.blackboard.SW_force=0;
                controller.blackboard.repulsion_tick=0;
                controller.blackboard.attraction_tick=0;


                controller.root->tick();

                if (controller.blackboard.forces !=0)
                {
                    if(controller.blackboard.NEWS_forces>0)
                    {
                        if ((controller.blackboard.NE_force==1 and controller.blackboard.SW_force==1 )
                            or (controller.blackboard.NW_force==1  and controller.blackboard.SE_force==1 ))
                        {
                            controller.blackboard.x_NEWS=0;
                            controller.blackboard.y_NEWS=0;
                        }
                        if (controller.blackboard.attraction_force==1 and controller.blackboard.repulsion_tick==0)
                        {
                            controller.blackboard.x_force+=(2.5)*(controller.blackboard.x_NEWS);
                            controller.blackboard.y_force+=(2.5)*(controller.blackboard.y_NEWS);

                        }
                        else
                        {
                            controller.blackboard.x_force+=(controller.blackboard.x_NEWS);
                            controller.blackboard.y_force+=(controller.blackboard.y_NEWS);
                        }
                    }


                    if (controller.blackboard.boundary_force==1)
                    {
                        if (controller.blackboard.attraction_force==1)
                        {
                            controller.blackboard.separation_force+=15;
                            //controller.blackboard.separation_decay = 0.0005;
                        }

                        else if (controller.blackboard.NEWS_forces>0)
                        {
                            controller.blackboard.separation_force+=15;
                        }

                    }

                    if (controller.blackboard.attraction_tick==0 or controller.blackboard.repulsion_tick==0)
                    {
                        controller.blackboard.x_force+=(controller.blackboard.x_neighbour_force);
                        controller.blackboard.y_force+=(controller.blackboard.y_neighbour_force);
                    }
                    else
                    {
                        controller.blackboard.separation_force=0;
                    }

                    if ((controller.blackboard.NE_force==1 and controller.blackboard.SW_force==1 )
                       or (controller.blackboard.NW_force==1  and controller.blackboard.SE_force==1 ))
                    {
                        controller.blackboard.separation_force=0;
                    }

                    if(controller.blackboard.attraction_force==1 and controller.blackboard.repulsion_tick==0)
                    {
                        controller.blackboard.separation_dist+=2;
                        controller.blackboard.separation_force += 5;

                    }
                    if (controller.blackboard.repulsion_tick==1 and controller.blackboard.attraction_tick==0)
                    {
                        controller.blackboard.separation_dist = 0;
                    }


                    if (controller.blackboard.separation_dist !=0)
                    {
                        double separation_force_x = 0.0;
                        double separation_force_y = 0.0;
                        double collided_robots = 0.0;
                        for (int k = 0; k < blackboard::Swarm_size; k++) {

                            if (Distances[j][k] < (2*blackboard::robot_radius)+controller.blackboard.separation_dist && k!=j) {

                                separation_force_x = separation_force_x + ( controller.blackboard.separation_force*exp(-Distances[j][k]*controller.blackboard.separation_decay) *
                                                        (controller.blackboard.DistancesX[j][k]/Distances[j][k]));
                                separation_force_y = separation_force_y + ( controller.blackboard.separation_force*exp(-Distances[j][k]*controller.blackboard.separation_decay) *
                                                        (controller.blackboard.DistancesY[j][k]/Distances[j][k]));

                                //separation_force_x = separation_force_x + (controller.blackboard.separation_force*5*exp(-Distances[j][k]/5)*controller.blackboard.DistancesX[j][k]);
                                //separation_force_y = separation_force_y + (controller.blackboard.separation_force*5*exp(-Distances[j][k]/5)*controller.blackboard.DistancesY[j][k]);
                                collided_robots += 1;

                            }
                        }
                        if (collided_robots>0)
                        {
                            controller.blackboard.x_force+=(separation_force_x);
                            controller.blackboard.y_force+=(separation_force_y);

                        }
                    }

                    if (controller.blackboard.agent.picked_object==true)
                    {
                        double separation_force_x = 0.0;
                        double separation_force_y = 0.0;
                        double collided_objects = 0.0;
                        double distance;
                        float separation_force_object=5;
                        float separation_dist_object=8;
                        float separation_decay_object=0.01;
                        for (int k = 0; k < controller.blackboard.BT_Env.objects_num; k++) {

                                if (controller.blackboard.BT_Env.objects_picked[k]==false)
                                {
                                    distance = sqrt(pow(controller.blackboard.agent.x - (controller.blackboard.BT_Env.objects_X[k]), 2) +
                                                    pow(controller.blackboard.agent.y - (controller.blackboard.BT_Env.objects_Y[k]), 2));
                                    if (distance < (2.5*blackboard::robot_radius)+separation_dist_object) {
                                        //separation_force_x = separation_force_x + (controller.blackboard.separation_force*10*exp(-distance/10)*(controller.blackboard.agent.x - (controller.blackboard.BT_Env.objects_X[k])));
                                        //separation_force_y = separation_force_y + (controller.blackboard.separation_force*10*exp(-distance/10)*(controller.blackboard.agent.y - (controller.blackboard.BT_Env.objects_Y[k])));

                                        separation_force_x = separation_force_x + ( separation_force_object*exp(-distance*separation_decay_object) *
                                                                                    ((controller.blackboard.agent.x - (controller.blackboard.BT_Env.objects_X[k]))/distance));
                                        separation_force_y = separation_force_y + ( separation_force_object*exp(-distance*separation_decay_object) *
                                                                                    ((controller.blackboard.agent.y - (controller.blackboard.BT_Env.objects_Y[k]))/distance));

                                        collided_objects += 1;

                                    }
                                }



                        }
                        if (collided_objects>0 and (controller.blackboard.x_force!=0 or controller.blackboard.y_force!=0))
                        {
                            controller.blackboard.x_force+=(separation_force_x);
                            controller.blackboard.y_force+=(separation_force_y);

                        }
                    }


                    if (controller.blackboard.x_force!=0 or controller.blackboard.y_force!=0)
                    {
                        heading =  atan2(controller.blackboard.y_force,controller.blackboard.x_force);
                        delta_x = blackboard::Swarm_speed*cos(heading)*blackboard::Swarm_time_step;
                        delta_y = blackboard::Swarm_speed*sin(heading)*blackboard::Swarm_time_step;
                        controller.blackboard.agent.x =  controller.blackboard.agent.x   + delta_x;
                        controller.blackboard.agent.y =  controller.blackboard.agent.y   + delta_y;
                    }





                    if (controller.blackboard.BT_Env.objects_in_env && controller.blackboard.agent.picked_object) {
                        controller.blackboard.BT_Env.objects_X[controller.blackboard.agent.object_ID] =
                                controller.blackboard.BT_Env.objects_X[controller.blackboard.agent.object_ID] + delta_x;
                        controller.blackboard.BT_Env.objects_Y[controller.blackboard.agent.object_ID] =
                                controller.blackboard.BT_Env.objects_Y[controller.blackboard.agent.object_ID] + delta_y;

                    }

                }
                bool agent_outside_area=true;
                if (controller.BT_environment.areas_in_env==true)
                {
                    for (int i = 0; i < controller.blackboard.BT_Env.areas_num; i++) {
                        if ((controller.blackboard.agent.x-controller.blackboard.robot_radius) >= (controller.blackboard.BT_Env.areas_X[i]) &&
                            (controller.blackboard.agent.x+controller.blackboard.robot_radius) <=
                            (controller.blackboard.BT_Env.areas_X[i] + controller.blackboard.BT_Env.areas_Width[i]) &&
                            (controller.blackboard.agent.y-controller.blackboard.robot_radius) >= (controller.blackboard.BT_Env.areas_Y[i]) &&
                            (controller.blackboard.agent.y+controller.blackboard.robot_radius) <=
                            (controller.blackboard.BT_Env.areas_Y[i] + controller.blackboard.BT_Env.areas_Width[i])) {
                            controller.blackboard.agent.inside_area= true;
                            agent_outside_area=false;



                        }
                    }
                }

                if(agent_outside_area== true)
                {
                    controller.blackboard.agent.inside_area= false;
                    controller.blackboard.agent.inside_area_t=0;
                }

                if(controller.blackboard.agent.inside_area== true)
                {
                    controller.blackboard.agent.inside_area_t+=1;
                }

                if(controller.blackboard.agent.picked_object== true)
                {
                    controller.blackboard.agent.picked_object_t+=1;
                }


                agent[j]=controller.blackboard.agent;





                SwarmX[i*blackboard::Swarm_size+j]=agent[j].x;
                SwarmY[i*blackboard::Swarm_size+j]=agent[j].y;
                x.push_back(agent[j].x);
                y.push_back(agent[j].y);

                Swarm_History2.push_back(agent[j]);

                picked_objs.push_back(controller.blackboard.Env_metrics.picked_obj_num);
                drop_objs.push_back(controller.blackboard.Env_metrics.drop_obj_num);
                in_area_times.push_back(controller.blackboard.Env_metrics.inside_area_times);





            }
            if(controller.blackboard.BT_Env.objects_in_env)
            {

                objects_Data_X.push_back(controller.blackboard.BT_Env.objects_X);
                objects_Data_Y.push_back(controller.blackboard.BT_Env.objects_Y);

            }

            controller.blackboard.Final_Trajectories_x.push_back(x);
            controller.blackboard.Final_Trajectories_y.push_back(y);
            x.clear();
            y.clear();

            compute_swarm_distance();
            std::copy(std::begin(agent), std::end(agent), std::begin(controller.blackboard.agents));




            //clear_distances ();

            //memcpy(AllDistances[i], controller.blackboard.Distances, sizeof(AllDistances[i]));
            Swarm_History.push_back(Swarm_History2);
            Swarm_History2.clear();
        }


    }

    void simulate_swarm(int swarm_size)
    {

        double delta_x = 0.0;
        double delta_y = 0.0;
        double heading =0.0;

        compute_swarm_distance(swarm_size);
        std::copy(std::begin(agent), std::end(agent), std::begin(controller.blackboard.agents));


        //memcpy(AllDistances[0], controller.blackboard.Distances, sizeof(AllDistances[0]));
        //AllDistances.clear();

        if(controller.blackboard.BT_Env.objects_in_env)
        {
            controller.blackboard.BT_Env.objects_X=controller.blackboard.BT_Env.Initial_objects_X;
            controller.blackboard.BT_Env.objects_Y=controller.blackboard.BT_Env.Initial_objects_Y;
            objects_Data_X.push_back(controller.blackboard.BT_Env.objects_X);
            objects_Data_Y.push_back(controller.blackboard.BT_Env.objects_Y);

        }
        controller.blackboard.Final_Trajectories_x.push_back(controller.blackboard.originalX);
        controller.blackboard.Final_Trajectories_y.push_back(controller.blackboard.originalY);

        for(int i = 1; i < blackboard::simulation_timesteps; i++)
        {
            vector<float> x;
            vector <float> y;
            controller.blackboard.boundary_force=0;
            for (int j=0; j<swarm_size;j++)
            {
                controller.blackboard.tick_times[j] += 1;
                //Node::agent =agent[i];
                //Node::agentID=i;
                controller.blackboard.agent=agent[j];
                controller.blackboard.agentID=j;

                controller.blackboard.x_force=0.0;
                controller.blackboard.y_force=0.0;
                controller.blackboard.x_NEWS=0.0;
                controller.blackboard.y_NEWS=0.0;
                controller.blackboard.x_neighbour_force=0.0;
                controller.blackboard.y_neighbour_force=0.0;
                controller.blackboard.forces=0;
                controller.blackboard.NEWS_forces=0;
                controller.blackboard.separation_dist=8;
                controller.blackboard.separation_force=5;
                controller.blackboard.separation_decay=0.01;
                controller.blackboard.move_backward=0;
                controller.blackboard.attraction_force=0;
                controller.blackboard.repulsion_force=0;
                controller.blackboard.NE_force=0;
                controller.blackboard.NW_force=0;
                controller.blackboard.SE_force=0;
                controller.blackboard.SW_force=0;
                controller.blackboard.repulsion_tick=0;
                controller.blackboard.attraction_tick=0;


                controller.root->tick();
                cout<<" agent "<<j;

                if (controller.blackboard.forces !=0)
                {
                    if(controller.blackboard.NEWS_forces>0)
                    {
                        if ((controller.blackboard.NE_force==1 and controller.blackboard.SW_force==1 )
                            or (controller.blackboard.NW_force==1  and controller.blackboard.SE_force==1 ))
                        {
                            controller.blackboard.x_NEWS=0;
                            controller.blackboard.y_NEWS=0;
                        }
                        if (controller.blackboard.attraction_force==1 and controller.blackboard.repulsion_tick==0)
                        {
                            controller.blackboard.x_force+=(2.5)*(controller.blackboard.x_NEWS);
                            controller.blackboard.y_force+=(2.5)*(controller.blackboard.y_NEWS);

                        }
                        else
                        {
                            controller.blackboard.x_force+=(controller.blackboard.x_NEWS);
                            controller.blackboard.y_force+=(controller.blackboard.y_NEWS);
                        }
                    }


                    if (controller.blackboard.boundary_force==1)
                    {
                        if (controller.blackboard.attraction_force==1)
                        {
                            controller.blackboard.separation_force+=15;
                            //controller.blackboard.separation_decay = 0.0005;
                        }

                        else if (controller.blackboard.NEWS_forces>0)
                        {
                            controller.blackboard.separation_force+=15;
                        }

                    }

                    if (controller.blackboard.attraction_tick==0 or controller.blackboard.repulsion_tick==0)
                    {
                        controller.blackboard.x_force+=(controller.blackboard.x_neighbour_force);
                        controller.blackboard.y_force+=(controller.blackboard.y_neighbour_force);
                    }
                    else
                    {
                        controller.blackboard.separation_force=0;
                    }

                    if ((controller.blackboard.NE_force==1 and controller.blackboard.SW_force==1 )
                        or (controller.blackboard.NW_force==1  and controller.blackboard.SE_force==1 ))
                    {
                        controller.blackboard.separation_force=0;
                    }

                    if(controller.blackboard.attraction_force==1 and controller.blackboard.repulsion_tick==0)
                    {
                        controller.blackboard.separation_dist+=2;
                        controller.blackboard.separation_force += 5;

                    }
                    if (controller.blackboard.repulsion_tick==1 and controller.blackboard.attraction_tick==0)
                    {
                        controller.blackboard.separation_dist = 0;
                    }


                    if (controller.blackboard.separation_dist !=0)
                    {
                        double separation_force_x = 0.0;
                        double separation_force_y = 0.0;
                        double collided_robots = 0.0;
                        for (int k = 0; k < swarm_size; k++) {

                            if (Distances[j][k] < (2*blackboard::robot_radius)+controller.blackboard.separation_dist && k!=j) {

                                separation_force_x = separation_force_x + ( controller.blackboard.separation_force*exp(-Distances[j][k]*controller.blackboard.separation_decay) *
                                                                            (controller.blackboard.DistancesX[j][k]/Distances[j][k]));
                                separation_force_y = separation_force_y + ( controller.blackboard.separation_force*exp(-Distances[j][k]*controller.blackboard.separation_decay) *
                                                                            (controller.blackboard.DistancesY[j][k]/Distances[j][k]));

                                //separation_force_x = separation_force_x + (controller.blackboard.separation_force*5*exp(-Distances[j][k]/5)*controller.blackboard.DistancesX[j][k]);
                                //separation_force_y = separation_force_y + (controller.blackboard.separation_force*5*exp(-Distances[j][k]/5)*controller.blackboard.DistancesY[j][k]);
                                collided_robots += 1;

                            }
                        }
                        if (collided_robots>0)
                        {
                            controller.blackboard.x_force+=(separation_force_x);
                            controller.blackboard.y_force+=(separation_force_y);

                        }
                    }

                    if (controller.blackboard.agent.picked_object==true)
                    {
                        double separation_force_x = 0.0;
                        double separation_force_y = 0.0;
                        double collided_objects = 0.0;
                        double distance;
                        float separation_force_object=5;
                        float separation_dist_object=8;
                        float separation_decay_object=0.01;
                        for (int k = 0; k < controller.blackboard.BT_Env.objects_num; k++) {

                            if (controller.blackboard.BT_Env.objects_picked[k]==false)
                            {
                                distance = sqrt(pow(controller.blackboard.agent.x - (controller.blackboard.BT_Env.objects_X[k]), 2) +
                                                pow(controller.blackboard.agent.y - (controller.blackboard.BT_Env.objects_Y[k]), 2));
                                if (distance < (2.5*blackboard::robot_radius)+separation_dist_object) {
                                    //separation_force_x = separation_force_x + (controller.blackboard.separation_force*10*exp(-distance/10)*(controller.blackboard.agent.x - (controller.blackboard.BT_Env.objects_X[k])));
                                    //separation_force_y = separation_force_y + (controller.blackboard.separation_force*10*exp(-distance/10)*(controller.blackboard.agent.y - (controller.blackboard.BT_Env.objects_Y[k])));

                                    separation_force_x = separation_force_x + ( separation_force_object*exp(-distance*separation_decay_object) *
                                                                                ((controller.blackboard.agent.x - (controller.blackboard.BT_Env.objects_X[k]))/distance));
                                    separation_force_y = separation_force_y + ( separation_force_object*exp(-distance*separation_decay_object) *
                                                                                ((controller.blackboard.agent.y - (controller.blackboard.BT_Env.objects_Y[k]))/distance));

                                    collided_objects += 1;

                                }
                            }



                        }
                        if (collided_objects>0 and (controller.blackboard.x_force!=0 or controller.blackboard.y_force!=0))
                        {
                            controller.blackboard.x_force+=(separation_force_x);
                            controller.blackboard.y_force+=(separation_force_y);

                        }
                    }


                    if (controller.blackboard.x_force!=0 or controller.blackboard.y_force!=0)
                    {
                        heading =  atan2(controller.blackboard.y_force,controller.blackboard.x_force);
                        delta_x = blackboard::Swarm_speed*cos(heading)*blackboard::Swarm_time_step;
                        delta_y = blackboard::Swarm_speed*sin(heading)*blackboard::Swarm_time_step;
                        controller.blackboard.agent.x =  controller.blackboard.agent.x   + delta_x;
                        controller.blackboard.agent.y =  controller.blackboard.agent.y   + delta_y;
                    }

                    cout<<" agent "<<j<<" controller.blackboard.BT_Env.objects_in_env "<<controller.blackboard.BT_Env.objects_in_env<<" controller.blackboard.agent.picked_object "
                    <<controller.blackboard.agent.picked_object<<" controller.blackboard.agent.object_ID "<<controller.blackboard.agent.object_ID<<endl;





                    if (controller.blackboard.BT_Env.objects_in_env and controller.blackboard.agent.picked_object) {
                        cout<<" controller.blackboard.agent.picked_object"<<controller.blackboard.agent.picked_object<<endl;
                        controller.blackboard.BT_Env.objects_X[controller.blackboard.agent.object_ID] =
                                controller.blackboard.BT_Env.objects_X[controller.blackboard.agent.object_ID] + delta_x;
                        controller.blackboard.BT_Env.objects_Y[controller.blackboard.agent.object_ID] =
                                controller.blackboard.BT_Env.objects_Y[controller.blackboard.agent.object_ID] + delta_y;

                    }

                }
                bool agent_outside_area=true;
                if (controller.BT_environment.areas_in_env==true)
                {
                    for (int i = 0; i < controller.blackboard.BT_Env.areas_num; i++) {
                        if ((controller.blackboard.agent.x-controller.blackboard.robot_radius) >= (controller.blackboard.BT_Env.areas_X[i]) &&
                            (controller.blackboard.agent.x+controller.blackboard.robot_radius) <=
                            (controller.blackboard.BT_Env.areas_X[i] + controller.blackboard.BT_Env.areas_Width[i]) &&
                            (controller.blackboard.agent.y-controller.blackboard.robot_radius) >= (controller.blackboard.BT_Env.areas_Y[i]) &&
                            (controller.blackboard.agent.y+controller.blackboard.robot_radius) <=
                            (controller.blackboard.BT_Env.areas_Y[i] + controller.blackboard.BT_Env.areas_Width[i])) {
                            controller.blackboard.agent.inside_area= true;
                            agent_outside_area=false;



                        }
                    }
                }

                if(agent_outside_area== true)
                {
                    controller.blackboard.agent.inside_area= false;
                    controller.blackboard.agent.inside_area_t=0;
                }

                if(controller.blackboard.agent.inside_area== true)
                {
                    controller.blackboard.agent.inside_area_t+=1;
                }

                if(controller.blackboard.agent.picked_object== true)
                {
                    controller.blackboard.agent.picked_object_t+=1;
                }


                agent[j]=controller.blackboard.agent;





                SwarmX[i*swarm_size+j]=agent[j].x;
                SwarmY[i*swarm_size+j]=agent[j].y;
                x.push_back(agent[j].x);
                y.push_back(agent[j].y);

                Swarm_History2.push_back(agent[j]);

                picked_objs.push_back(controller.blackboard.Env_metrics.picked_obj_num);
                drop_objs.push_back(controller.blackboard.Env_metrics.drop_obj_num);
                in_area_times.push_back(controller.blackboard.Env_metrics.inside_area_times);





            }
            if(controller.blackboard.BT_Env.objects_in_env)
            {

                objects_Data_X.push_back(controller.blackboard.BT_Env.objects_X);
                objects_Data_Y.push_back(controller.blackboard.BT_Env.objects_Y);

            }

            controller.blackboard.Final_Trajectories_x.push_back(x);
            controller.blackboard.Final_Trajectories_y.push_back(y);
            x.clear();
            y.clear();

            compute_swarm_distance(swarm_size);
            std::copy(std::begin(agent), std::end(agent), std::begin(controller.blackboard.agents));




            //clear_distances ();

            //memcpy(AllDistances[i], controller.blackboard.Distances, sizeof(AllDistances[i]));
            Swarm_History.push_back(Swarm_History2);
            Swarm_History2.clear();
        }


    }


};

#endif //SWARM_SWARM_H
