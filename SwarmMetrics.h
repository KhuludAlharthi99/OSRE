//
// Created by kholood alharthi on 10/20/21.
//
#include <iostream>
using namespace std;
#include <sstream>
#include "Swarm.h"
#include "blackboard.h"
#include <iostream>
#include "rapidcsv.h"
#include "csv2.h"
#include <string>
#include<stdio.h>
#include<stdlib.h>
#include <numeric>

using namespace csv;





#ifndef SWARM_SWARMMETRICS_H
#define SWARM_SWARMMETRICS_H


class SwarmMetrics {

public:
    Swarm Sim1;

    vector<vector<float>> swarm_metrics={};
    vector<vector<float>> swarm_metrics_features={};
    vector<vector<float>> swarm_trajectories={};

    vector<float> centerMassX ={};
    vector<float> centerMassY={};

    vector<float> max_radius={};

    vector<float> NNdistAvg={};
    vector<float> NNdist={};
    vector<float> local_density_avg={};
    vector<float> local_density={};


    vector<float> BetaIndex={};
    vector<float> Local_degrees_mode={};


    vector<float> AvgVelX={};
    vector<float> AvgVelY={};

    vector<float> CoMVelX={};
    vector<float> CoMVelY={};

    vector<float> SDVelX={};
    vector<float> SDVelY={};

    vector<float> MaxVelX={};
    vector<float> MaxVelY={};

    vector<float> MinVelX={};
    vector<float> MinVelY={};

    vector<float> AgentVelX={};
    vector<float> AgentVelY={};

    vector<float> longest_path={};

    vector <float> Mode_X={};
    vector <float> Mode_Y={};
    vector <float> Idle_agents={};

    vector <float> agents_inside_area_ratio={};
    vector <float> visited_areas_ratio_cur={};
    vector <float> areas_Beta={};
    vector <float> avg_NN_VelX={};
    vector <float> avg_NN_VelY={};
    vector <float> areas_avg_NN={};
    vector <float> areas_avg_FN={};

    vector<float> objects_centerMassX ={};
    vector<float> objects_centerMassY={};
    vector<float> objects_AvgVelX={};
    vector<float> objects_AvgVelY={};
    vector<float> objects_picked={};
    vector<float> objects_dropped={};
    vector<float> objects_Beta={};
    vector<float> objects_paths_ratios={};
    vector<float> objects_avg_NSD={};

    vector <float> objects_inside_area_ratio={};
    vector <float> areas_Beta2={};
    vector <float> avg_NN_VelX2={};
    vector <float> avg_NN_VelY2={};
    vector <float> areas_avg_NN2={};
    vector <float> areas_avg_FN2={};

    float objects_agents_avg_distance;
    float areas_agents_avg_distance;


    vector<vector <float>> agent_level_disp_X={};
    vector<vector <float>>  agent_level_disp_Y={};
    vector<vector <float>>  agent_level_area_dist={};
    vector<vector <float>>  agent_level_objects_dist={};

    vector <float> agent_degree={};
    vector <float> edge_degree_avg={};

    vector <float> swarm_area_density={};
    vector <float> avg_clustering_coeff={};
    vector <float> clustering_coeff={};
    vector<vector<double>> trajectory1 ={};
    vector<vector<double>> trajectory2 ={};
    vector<int> swarm_sizes={};

    //vector<vector<float>> original_x ={};
    //vector<vector<float>> original_y ={};

    int id1,id2;
    float AvgVelX_all,AvgVelY_all;







    void set_sizes ()
    {


        trajectory1.clear();
        trajectory2.clear();

        swarm_metrics.clear();
        swarm_metrics_features.clear();
        centerMassX.clear();
        centerMassY.clear();
        max_radius.clear();

        NNdistAvg.clear();
        local_density_avg.clear();

        BetaIndex.clear();
        //longest_path.clear();

        AvgVelX.clear();
        AvgVelY.clear();

        MaxVelX.clear();
        MaxVelY.clear();

        SDVelX.clear();
        SDVelY.clear();

        Local_degrees_mode.clear();
        centerMassX.clear();
        centerMassY.clear();
        max_radius.clear();

        NNdistAvg.clear();
        NNdist.clear();
        local_density_avg.clear();
        local_density.clear();


        BetaIndex.clear();
        longest_path.clear();

        AvgVelX.clear();
        AvgVelY.clear();

        MaxVelX.clear();
        MaxVelY.clear();

        AgentVelX.clear();
        AgentVelY.clear();

        CoMVelX.clear();
        CoMVelY.clear();

        SDVelX.clear();
        SDVelY.clear();

        Local_degrees_mode.clear();
        Idle_agents.clear();

        agent_degree.clear();
        edge_degree_avg.clear();

        swarm_area_density.clear();
        avg_clustering_coeff.clear();
        clustering_coeff.clear();

        agents_inside_area_ratio.clear();
        visited_areas_ratio_cur.clear();
        avg_NN_VelX.clear();
        avg_NN_VelX.clear();
        areas_Beta.clear();
        areas_avg_NN.clear();
        areas_avg_FN.clear();


        objects_centerMassX.clear();
        objects_centerMassY.clear();
        objects_AvgVelX.clear();
        objects_AvgVelY.clear();
        objects_picked.clear();
        objects_dropped.clear();
        objects_Beta.clear();
        objects_avg_NSD.clear();
        objects_paths_ratios.clear();

        objects_inside_area_ratio.clear();
        areas_Beta2.clear();
        avg_NN_VelX2.clear();
        avg_NN_VelY2.clear();
        areas_avg_NN2.clear();
        areas_avg_FN2.clear();

        agent_level_disp_X.clear();
        agent_level_disp_Y.clear();
        agent_level_area_dist.clear();
        agent_level_objects_dist.clear();


        for(int i=0;i<blackboard::simulation_timesteps;i++)
        {



            centerMassX.push_back(0.0);
            centerMassY.push_back(0.0);
            max_radius.push_back(0.0);

            NNdistAvg.push_back(0.0);
            NNdist.push_back(0.0);


            local_density_avg.push_back(0.0);
            local_density.push_back(0.0);

            BetaIndex.push_back(0.0);
            longest_path.push_back(0.0);

            AvgVelX.push_back(0.0);
            AvgVelY.push_back(0.0);

            MaxVelX.push_back(0.0);
            MaxVelY.push_back(0.0);

            MinVelX.push_back(0.0);
            MinVelY.push_back(0.0);

            AgentVelX.push_back(0.0);
            AgentVelY.push_back(0.0);

            CoMVelX.push_back(0.0);
            CoMVelY.push_back(0.0);

            SDVelX.push_back(0.0);
            SDVelY.push_back(0.0);

            Local_degrees_mode.push_back(0.0);
            agent_degree.push_back(0.0);
            edge_degree_avg.push_back(0.0);

            Idle_agents.push_back(0.0);

            swarm_area_density.push_back(0.0);
            avg_clustering_coeff.push_back(0.0);
            clustering_coeff.push_back(0.0);

            agents_inside_area_ratio.push_back(0.0);
            visited_areas_ratio_cur.push_back(0.0);
            avg_NN_VelX.push_back(0.0);
            avg_NN_VelY.push_back(0.0);
            areas_Beta.push_back(0.0);
            areas_avg_NN.push_back(0.0);
            areas_avg_FN.push_back(0.0);

            objects_centerMassX.push_back(0.0);
            objects_centerMassY.push_back(0.0);
            objects_AvgVelX.push_back(0.0);
            objects_AvgVelY.push_back(0.0);
            objects_picked.push_back(0.0);
            objects_dropped.push_back(0.0);
            objects_Beta.push_back(0.0);
            objects_avg_NSD.push_back(0.0);
            objects_paths_ratios.push_back(0.0);

            objects_inside_area_ratio.push_back(0.0);
            areas_Beta2.push_back(0.0);
            avg_NN_VelX2.push_back(0.0);
            avg_NN_VelY2.push_back(0.0);
            areas_avg_NN2.push_back(0.0);
            areas_avg_FN2.push_back(0.0);


        }


    }

    void reset ()
    {
        Sim1.reset();
        set_sizes();
    }

    void reset (int swarm_size)
    {
        Sim1.reset(swarm_size);
        set_sizes();
    }


    void generate_simulation_data ( )
    {
       // Sim1.BT1=BTa;
           //Sim1.read_initialize_swarm();
           Sim1.generate_swarm();
           Sim1.simulate_swarm();

    }

    void generate_simulation_data (int swarm_size )
    {
        // Sim1.BT1=BTa;
        //Sim1.read_initialize_swarm();
        Sim1.generate_swarm(swarm_size);
        Sim1.simulate_swarm(swarm_size);

    }

    void generate_simulation_data2 (int swarm_size)
    {
        // Sim1.BT1=BTa;

        Sim1.initialize_swarm(swarm_size);
        Sim1.simulate_swarm(swarm_size);

    }

    void generate_simulation_data2 ()
    {
        // Sim1.BT1=BTa;

            Sim1.initialize_swarm();
            Sim1.simulate_swarm();

    }

    void generate_simulation_data3 ()
    {
        // Sim1.BT1=BTa;

        Sim1.initialize_swarm2();
        Sim1.simulate_swarm();

    }

    void generate_simulation_data3 (int swarm_size)
    {
        // Sim1.BT1=BTa;

        Sim1.initialize_swarm2(swarm_size);
        Sim1.simulate_swarm(swarm_size);

    }


    void generate_simulation_data4()
    {
        // Sim1.BT1=BTa;

        Sim1.initialize_swarm3();
        Sim1.simulate_swarm();

    }


    double customRound(double num, double threshold) {
        if (std::abs(num) < threshold) {
            // Round down
            return std::floor(num);
        } else {
            // Round up
            return std::ceil(num);
        }
    }


    void center_displacement ()
    {
        CoMVelX[0]=0;
        CoMVelY[0]=0;
        for (long i = 1; i <  blackboard::simulation_timesteps; i++)
        {

            CoMVelX[i]=(centerMassX[i]-centerMassX[i-1]);
            CoMVelY[i]=(centerMassY[i]-centerMassY[i-1]);;

        }

        swarm_metrics.push_back(CoMVelX);
        swarm_metrics.push_back(CoMVelY);
    }

    void agents_level_metrics (bool area, bool objects)
    {
        float dist_temp,NNdist;

        for (int j = 0; j < blackboard::Swarm_size ; j++)
        {
            vector<float> velx;
            vector<float> vely;
            vector<float> nearest_area;
            vector<float> nearest_object;

           for (long i = 1; i <  blackboard::simulation_timesteps; i++)
           {
               velx.push_back(Sim1.SwarmX[i*blackboard::Swarm_size+j]-Sim1.SwarmX[(i-1)*blackboard::Swarm_size+j]);
               vely.push_back(Sim1.SwarmY[i*blackboard::Swarm_size+j]-Sim1.SwarmY[(i-1)*blackboard::Swarm_size+j]);

               NNdist = 100000;
               if (area)
               {
                   for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++) {
                       dist_temp = sqrt(pow((Sim1.SwarmX[i*blackboard::Swarm_size+j])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.SwarmY[i*blackboard::Swarm_size+j])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                       if (dist_temp < NNdist )
                       {
                           NNdist= dist_temp;
                       }
                   }

                   nearest_area.push_back(NNdist);
               }

               NNdist = 100000;
               if (objects)
               {
                   for (int a = 0; a < Sim1.controller.blackboard.BT_Env.objects_num; a++) {
                       dist_temp = sqrt(pow((Sim1.SwarmX[i*blackboard::Swarm_size+j])  - (Sim1.objects_Data_X[i][a]), 2) + pow((Sim1.SwarmY[i*blackboard::Swarm_size+j])- (Sim1.objects_Data_Y[i][a]), 2));
                       if (dist_temp < NNdist )
                       {
                           NNdist= dist_temp;
                       }
                   }

                   nearest_object.push_back(NNdist);
               }

           }

            agent_level_disp_X.push_back(velx);
            agent_level_disp_Y.push_back(vely);
            if (area)
            {
                agent_level_area_dist.push_back(nearest_area);
            }
            if (objects)
            {
                agent_level_objects_dist.push_back(nearest_object);
            }


        }


    }

    void furthest_agents_id ()
    {
        double path =0.0;
        for (int j = 0; j < blackboard::Swarm_size; j++)
        {
            for (int k = j+1; k < blackboard::Swarm_size; k++)
            {
                if (Sim1.AllDistances[0][j][k] > path )
                {
                    path=Sim1.AllDistances[0][j][k] ;
                    id1=j;
                    id2=k;

                }
            }
        }

    }

    void compute_metrics_features ()
    {



        for (int s=0; s<swarm_metrics.size(); s++)
        {
            vector <float> features={};
            float average = accumulate(swarm_metrics[s].begin(), swarm_metrics[s].end(), 0.0) / swarm_metrics[s].size();
            float abs_energy=0;
            float abs_sum_of_changes=0;
            float count_above_mean=0;
            float count_below_mean=0;
            // abs_energy
            for(int i=0;i<blackboard::simulation_timesteps;i++) {
                abs_energy=abs_energy+(swarm_metrics[s][i]*swarm_metrics[s][i]);
                if (i+1!=blackboard::simulation_timesteps)
                {
                    abs_sum_of_changes=abs_sum_of_changes+abs(swarm_metrics[s][i+1]-swarm_metrics[s][i]);
                }
                if (swarm_metrics[s][i] > average)
                {
                    count_above_mean+=1;
                }
                if (swarm_metrics[s][i] < average)
                {
                    count_below_mean+=1;
                }
            }
            features.push_back(abs_energy);
            features.push_back(abs_sum_of_changes);
            features.push_back(count_above_mean);
            features.push_back(count_below_mean);

            swarm_metrics_features.push_back(features);

        }

    }

    void agents_trajectories (int agent1, int agent2)
    {

        vector<float> x={};
        vector<float> y={};

        vector<float> x2={};
        vector<float> y2={};

            for  (int i=0; i<blackboard::simulation_timesteps;i++)
            {
                //trajectory1.push_back({Sim1.SwarmX[i*blackboard::Swarm_size+agent1],Sim1.SwarmY[i*blackboard::Swarm_size+agent1]});
                //trajectory2.push_back({Sim1.SwarmX[i*blackboard::Swarm_size+agent2],Sim1.SwarmY[i*blackboard::Swarm_size+agent2]});
                x.push_back(Sim1.SwarmX[i*blackboard::Swarm_size+agent1]);
                y.push_back(Sim1.SwarmY[i*blackboard::Swarm_size+agent1]);
                x2.push_back(Sim1.SwarmX[i*blackboard::Swarm_size+agent2]);
                y2.push_back(Sim1.SwarmY[i*blackboard::Swarm_size+agent2]);
            }

        swarm_trajectories.push_back(x);
        swarm_trajectories.push_back(y);

        swarm_trajectories.push_back(x2);
        swarm_trajectories.push_back(y2);

    }

    void agents_trajectories (int agent_num)
    {
        for (int a=0;a<agent_num;a++)
        {
            vector<float> x={};
            vector<float> y={};


            for  (int i=0; i<blackboard::simulation_timesteps;i++)
            {
                //trajectory1.push_back({Sim1.SwarmX[i*blackboard::Swarm_size+agent1],Sim1.SwarmY[i*blackboard::Swarm_size+agent1]});
                //trajectory2.push_back({Sim1.SwarmX[i*blackboard::Swarm_size+agent2],Sim1.SwarmY[i*blackboard::Swarm_size+agent2]});
                x.push_back(Sim1.SwarmX[i*blackboard::Swarm_size+a]);
                y.push_back(Sim1.SwarmY[i*blackboard::Swarm_size+a]);

            }

            swarm_trajectories.push_back(x);
            swarm_trajectories.push_back(y);

        }



    }

    void center_of_mass ()
    {
        for (int i=0; i<blackboard::simulation_timesteps;i++)
        {
            double x_center=0;
            double y_center=0;

            for (int j=0; j< blackboard::Swarm_size;j++)
            {
                x_center=x_center+Sim1.SwarmX[i*blackboard::Swarm_size+j];
                y_center=y_center+Sim1.SwarmY[i*blackboard::Swarm_size+j];
            }

            centerMassX[i]= x_center/ double(blackboard::Swarm_size);
            centerMassY[i]= y_center/ double(blackboard::Swarm_size);

        }

        swarm_metrics.push_back(centerMassX);
        swarm_metrics.push_back(centerMassY);
    }

    void center_of_mass (int swarm_size)
    {
        for (int i=0; i<blackboard::simulation_timesteps;i++)
        {
            double x_center=0;
            double y_center=0;

            for (int j=0; j< swarm_size;j++)
            {
                x_center=x_center+Sim1.SwarmX[i*swarm_size+j];
                y_center=y_center+Sim1.SwarmY[i*swarm_size+j];
            }

            centerMassX[i]= x_center/ double(swarm_size);
            centerMassY[i]= y_center/ double(swarm_size);

        }

        swarm_metrics.push_back(centerMassX);
        swarm_metrics.push_back(centerMassY);
    }

    void max_Radius ()
    {
        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            double dist=0.0;

            for (int j = 0; j < blackboard::Swarm_size; j++)
            {
                    dist = sqrt(pow(centerMassX[i]-Sim1.SwarmX[i*blackboard::Swarm_size+j],2)+pow(centerMassY[i]-Sim1.SwarmY[i*blackboard::Swarm_size+j],2));
                    if (dist> max_radius[i] )
                        max_radius[i]= dist;
            }
        }
        swarm_metrics.push_back(max_radius);

    }

    void max_Radius (int swarm_size)
    {
        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            double dist=0.0;

            for (int j = 0; j < swarm_size; j++)
            {
                dist = sqrt(pow(centerMassX[i]-Sim1.SwarmX[i*swarm_size+j],2)+pow(centerMassY[i]-Sim1.SwarmY[i*swarm_size+j],2));
                if (dist> max_radius[i] )
                    max_radius[i]= dist;
            }
        }
        swarm_metrics.push_back(max_radius);

    }

    void local_Density_avg ()
    {
        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            double sumLocalDens = 0;

            for (int j = 0; j <  blackboard::Swarm_size; j++)
            {
                for (int  k= 0; k <  blackboard::Swarm_size; k++)
                {
                    if (Sim1.AllDistances[i][j][k] <= blackboard::local_radius && Sim1.AllDistances[i][j][k]>0)
                    {
                        //localDens[i][j]+=1;
                        sumLocalDens +=1;
                    }
                }
            }
            local_density_avg[i] = (sumLocalDens) / double(blackboard::Swarm_size);
        }
        swarm_metrics.push_back(local_density_avg);

    }

    void local_Density (int agent_id)
    {
        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            double sumLocalDens = 0;


                for (int  k= 0; k <  blackboard::Swarm_size; k++)
                {
                    if (Sim1.AllDistances[i][agent_id][k] <= blackboard::local_radius && Sim1.AllDistances[i][agent_id][k]>0)
                    {
                        sumLocalDens +=1;
                    }
                }

            local_density[i] = (sumLocalDens) ;
        }
        swarm_metrics.push_back(local_density);

    }

    void Edge_Degree (int agent_id)
    {
        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            double sumLocalDens = 0;


            for (int  k= 0; k <  blackboard::Swarm_size; k++)
            {
                if (Sim1.AllDistances[i][agent_id][k] <= blackboard::local_radius && Sim1.AllDistances[i][agent_id][k]>0)
                {
                    sumLocalDens +=1;
                }
            }

            agent_degree[i] = (sumLocalDens/local_density_avg[i]) ;
        }
        swarm_metrics.push_back(agent_degree);

    }

    void avg_Edge_Degree ( )
    {
        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            double edge_degree_sum =0.0;

            for (int j = 0; j < blackboard::Swarm_size; j++) {
                double sumLocalDens = 0;
                for (int k = 0; k < blackboard::Swarm_size; k++) {
                    if (Sim1.AllDistances[i][j][k] <= blackboard::local_radius &&
                        Sim1.AllDistances[i][j][k] > 0) {
                        sumLocalDens += 1;
                    }
                }
                edge_degree_sum += (sumLocalDens/local_density_avg[i]);
            }

            edge_degree_avg[i] = (edge_degree_sum/double(blackboard::Swarm_size)) ;
        }
        swarm_metrics.push_back(edge_degree_avg);

    }

    void NN_distance_avg ()
    {
        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            double sumNNdist = 0;
            float NNdist;

            for (int j = 0; j < blackboard::Swarm_size; j++)
            {
                NNdist = 100000;
                for (int k=0; k< blackboard::Swarm_size; k++)
                {
                    if (Sim1.AllDistances[i][j][k] < NNdist && Sim1.AllDistances[i][j][k]>0  )
                        NNdist= Sim1.AllDistances[i][j][k];
                }

                sumNNdist =  sumNNdist + NNdist ;
            }
            NNdistAvg[i] = sumNNdist/ double(blackboard::Swarm_size);
        }

        swarm_metrics.push_back(NNdistAvg);
    }

    void NN_distance_avg (int swarm_size)
    {
        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            double sumNNdist = 0;
            float NNdist;

            for (int j = 0; j < swarm_size; j++)
            {
                NNdist = 100000;
                for (int k=0; k< swarm_size; k++)
                {
                    if (Sim1.AllDistances[i][j][k] < NNdist && Sim1.AllDistances[i][j][k]>0  )
                        NNdist= Sim1.AllDistances[i][j][k];
                }

                sumNNdist =  sumNNdist + NNdist ;
            }
            NNdistAvg[i] = sumNNdist/ double(swarm_size);
        }

        swarm_metrics.push_back(NNdistAvg);
    }

    void NN_distance (int agent_id)
    {
        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            float dist;

            dist = 100000;
            for (int k=0; k< blackboard::Swarm_size; k++)
            {
                if (Sim1.AllDistances[i][agent_id][k] < dist && Sim1.AllDistances[i][agent_id][k]>0  )
                    dist= Sim1.AllDistances[i][agent_id][k];
            }


            NNdist[i] = dist;
        }

        swarm_metrics.push_back(NNdist);
    }


    void longest_Path ()
    {
        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            double path =0.0;
            for (int j = 0; j < blackboard::Swarm_size; j++)
            {
                for (int k = j+1; k < blackboard::Swarm_size; k++)
                {
                    if (Sim1.AllDistances[i][j][k] > path )
                    {
                        path=Sim1.AllDistances[i][j][k] ;

                    }
                }
            }
            longest_path[i]=path;
        }

        swarm_metrics.push_back(longest_path);

    }

    void longest_Path (int swarm_size)
    {
        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            double path =0.0;
            for (int j = 0; j < swarm_size; j++)
            {
                for (int k = j+1; k < swarm_size; k++)
                {
                    if (Sim1.AllDistances[i][j][k] > path )
                    {
                        path=Sim1.AllDistances[i][j][k] ;

                    }
                }
            }
            longest_path[i]=path;
        }

        swarm_metrics.push_back(longest_path);

    }

      int getRoot(int root[], int i) {
        while (root[i] != i) {
            root[i] = root[root[i]];
            i = root[i];
        }
        return i;
    }


    void max_displacement ()
    {
        MaxVelX[0]=0;
        MaxVelY[0]=0;
        for (long i = 1; i <  blackboard::simulation_timesteps; i++)
        {
            float velx= 0.0;
            float vely= 0.0;
            float r_velx= 0.0;
            float r_vely= 0.0;

            for (int j = 0; j < blackboard::Swarm_size ; j++)
            {
                if (abs((Sim1.SwarmX[i*blackboard::Swarm_size+j]-Sim1.SwarmX[(i-1)*blackboard::Swarm_size+j]))>velx)
                {
                    velx=abs((Sim1.SwarmX[i*blackboard::Swarm_size+j]-Sim1.SwarmX[(i-1)*blackboard::Swarm_size+j]));
                    r_velx =(Sim1.SwarmX[i*blackboard::Swarm_size+j]-Sim1.SwarmX[(i-1)*blackboard::Swarm_size+j]);
                }
                if (abs((Sim1.SwarmY[i*blackboard::Swarm_size+j]-Sim1.SwarmY[(i-1)*blackboard::Swarm_size+j]))>vely)
                {
                    vely=abs((Sim1.SwarmY[i*blackboard::Swarm_size+j]-Sim1.SwarmY[(i-1)*blackboard::Swarm_size+j]));
                    r_vely=(Sim1.SwarmY[i*blackboard::Swarm_size+j]-Sim1.SwarmY[(i-1)*blackboard::Swarm_size+j]);
                }
            }

            MaxVelX[i]=r_velx;
            MaxVelY[i]=r_vely;

        }

        swarm_metrics.push_back(MaxVelX);
        swarm_metrics.push_back(MaxVelY);


    }

    void avg_displacement ()
    {
        AvgVelX[0]=0;
        AvgVelY[0]=0;
        for (long i = 1; i <  blackboard::simulation_timesteps; i++)
        {
            float velx= 0.0;
            float vely= 0.0;
            float r_velx= 0.0;
            float r_vely= 0.0;

            for (int j = 0; j < blackboard::Swarm_size ; j++)
            {
                velx+= (Sim1.SwarmX[i*blackboard::Swarm_size+j]-Sim1.SwarmX[(i-1)*blackboard::Swarm_size+j]);
                vely+= (Sim1.SwarmY[i*blackboard::Swarm_size+j]-Sim1.SwarmY[(i-1)*blackboard::Swarm_size+j]);
            }

            AvgVelX[i]=(velx/double(blackboard::Swarm_size));
            AvgVelY[i]=(vely/double(blackboard::Swarm_size));

        }

        swarm_metrics.push_back(AvgVelX);
        swarm_metrics.push_back(AvgVelY);


    }

    void avg_displacement (int swarm_size)
    {
        AvgVelX[0]=0;
        AvgVelY[0]=0;
        for (long i = 1; i <  blackboard::simulation_timesteps; i++)
        {
            float velx= 0.0;
            float vely= 0.0;
            float r_velx= 0.0;
            float r_vely= 0.0;

            for (int j = 0; j < swarm_size ; j++)
            {
                velx+= (Sim1.SwarmX[i*swarm_size+j]-Sim1.SwarmX[(i-1)*swarm_size+j]);
                vely+= (Sim1.SwarmY[i*swarm_size+j]-Sim1.SwarmY[(i-1)*swarm_size+j]);
            }

            AvgVelX[i]=(velx/double(swarm_size));
            AvgVelY[i]=(vely/double(swarm_size));

        }

        swarm_metrics.push_back(AvgVelX);
        swarm_metrics.push_back(AvgVelY);


    }

    void avg_displacement_all (int timesteps)
    {
        float AvgVelX=0;
        float AvgVelY=0;
        for (int i = 1; i <  timesteps; i++)
        {
            float velx= 0.0;
            float vely= 0.0;
            float r_velx= 0.0;
            float r_vely= 0.0;

            for (int j = 0; j < blackboard::Swarm_size ; j++)
            {
                velx+= abs(Sim1.SwarmX[i*blackboard::Swarm_size+j]-Sim1.SwarmX[(i-1)*blackboard::Swarm_size+j]);
                vely+= abs(Sim1.SwarmY[i*blackboard::Swarm_size+j]-Sim1.SwarmY[(i-1)*blackboard::Swarm_size+j]);
            }

            AvgVelX+=(velx/float(blackboard::Swarm_size));
            AvgVelY+=(vely/float(blackboard::Swarm_size));

        }

        AvgVelX_all= (AvgVelX/float(blackboard::simulation_timesteps-1));
        AvgVelY_all= (AvgVelY/float(blackboard::simulation_timesteps-1));



    }

    void avg_displacement_all ()
    {
        float AvgVelX=0;
        float AvgVelY=0;
        for (int i = 1; i <  blackboard::simulation_timesteps; i++)
        {
            float velx= 0.0;
            float vely= 0.0;
            float r_velx= 0.0;
            float r_vely= 0.0;

            for (int j = 0; j < blackboard::Swarm_size ; j++)
            {
                velx+= abs(Sim1.SwarmX[i*blackboard::Swarm_size+j]-Sim1.SwarmX[(i-1)*blackboard::Swarm_size+j]);
                vely+= abs(Sim1.SwarmY[i*blackboard::Swarm_size+j]-Sim1.SwarmY[(i-1)*blackboard::Swarm_size+j]);
            }

            AvgVelX+=(velx/float(blackboard::Swarm_size));
            AvgVelY+=(vely/float(blackboard::Swarm_size));

        }

        AvgVelX_all= (AvgVelX/float(blackboard::simulation_timesteps-1));
        AvgVelY_all= (AvgVelY/float(blackboard::simulation_timesteps-1));



    }

    void avg_displacement_all2 (int swarm_size)
    {
        float AvgVelX=0;
        float AvgVelY=0;
        for (int i = 1; i <  blackboard::simulation_timesteps; i++)
        {
            float velx= 0.0;
            float vely= 0.0;
            float r_velx= 0.0;
            float r_vely= 0.0;

            for (int j = 0; j < swarm_size ; j++)
            {
                velx+= abs(Sim1.SwarmX[i*swarm_size+j]-Sim1.SwarmX[(i-1)*swarm_size+j]);
                vely+= abs(Sim1.SwarmY[i*swarm_size+j]-Sim1.SwarmY[(i-1)*swarm_size+j]);
            }

            AvgVelX+=(velx/float(swarm_size));
            AvgVelY+=(vely/float(swarm_size));

        }

        AvgVelX_all= (AvgVelX/float(blackboard::simulation_timesteps-1));
        AvgVelY_all= (AvgVelY/float(blackboard::simulation_timesteps-1));



    }

    void agent_displacement (int agent_id)
    {
        AgentVelX[0]=0;
        AgentVelY[0]=0;
        for (int i = 1; i <  blackboard::simulation_timesteps; i++)
        {

            AgentVelX[i]=((Sim1.SwarmX[i*blackboard::Swarm_size+agent_id]-Sim1.SwarmX[(i-1)*blackboard::Swarm_size+agent_id]));
            AgentVelY[i]=((Sim1.SwarmY[i*blackboard::Swarm_size+agent_id]-Sim1.SwarmY[(i-1)*blackboard::Swarm_size+agent_id]));

        }

        swarm_metrics.push_back(AgentVelX);
        swarm_metrics.push_back(AgentVelY);


    }

    void min_displacement ()
    {
        MinVelX[0]=0;
        MinVelY[0]=0;
        for (int i = 1; i <  blackboard::simulation_timesteps; i++)
        {
            float velx= 5000;
            float vely= 5000;
            float r_velx= 0.0;
            float r_vely= 0.0;

            for (int j = 0; j < blackboard::Swarm_size ; j++)
            {
                if (abs((Sim1.SwarmX[i*blackboard::Swarm_size+j]-Sim1.SwarmX[(i-1)*blackboard::Swarm_size+j]))<velx)
                {
                    velx=abs((Sim1.SwarmX[i*blackboard::Swarm_size+j]-Sim1.SwarmX[(i-1)*blackboard::Swarm_size+j]));
                    r_velx =(Sim1.SwarmX[i*blackboard::Swarm_size+j]-Sim1.SwarmX[(i-1)*blackboard::Swarm_size+j]);
                }
                if (abs((Sim1.SwarmY[i*blackboard::Swarm_size+j]-Sim1.SwarmY[(i-1)*blackboard::Swarm_size+j]))<vely)
                {
                    vely=abs((Sim1.SwarmY[i*blackboard::Swarm_size+j]-Sim1.SwarmY[(i-1)*blackboard::Swarm_size+j]));
                    r_vely=(Sim1.SwarmY[i*blackboard::Swarm_size+j]-Sim1.SwarmY[(i-1)*blackboard::Swarm_size+j]);
                }
            }

            MinVelX[i]=r_velx;
            MinVelY[i]=r_vely;

        }

        swarm_metrics.push_back(MinVelX);
        swarm_metrics.push_back(MinVelY);


    }

    void spatial_Frequency ()
    {

        float EPSILON = 0.1;
        int index_X = 0;
        int index_Y = 0;

        double highest_X = 0.0;
        double highest_Y = 0.0;

        for (int i =0 ; i <  blackboard::simulation_timesteps; i++)
        {
            for ( int a = 0; a < blackboard::Swarm_size; a++)
            {
                int count_X = 0;
                int count_Y = 0;

                double Position_X = Sim1.SwarmX[i*blackboard::Swarm_size+a];
                double Position_Y = Sim1.SwarmY[i*blackboard::Swarm_size+a];

                for (unsigned int b = a + 1; b < blackboard::Swarm_size; b++)
                {
                    if  (((Sim1.SwarmX[i*blackboard::Swarm_size+b] - Position_X) < EPSILON) && ((Position_X - Sim1.SwarmX[i*blackboard::Swarm_size+b]) < EPSILON))
                    {
                        count_X++;
                    }

                    if  (((Sim1.SwarmY[i*blackboard::Swarm_size+b] - Position_Y) < EPSILON) && ((Position_Y - Sim1.SwarmY[i*blackboard::Swarm_size+b]) < EPSILON))
                    {
                        count_Y++;
                    }
                }
                if (count_X >= index_X)
                {
                    index_X = count_X;
                    highest_X = Position_X;
                }

                if (count_Y >= index_Y)
                {
                    index_Y = count_Y;
                    highest_Y = Position_Y;
                }
            }

            Mode_X.push_back(highest_X);
            Mode_Y.push_back(highest_Y);

        }

        swarm_metrics.push_back(Mode_X);
        swarm_metrics.push_back(Mode_Y);

    }


    void betaIndex ()
    {

        int count = 0;
        double avgDist = 0.0;

            for (int j = 0; j < blackboard::Swarm_size; j++) {
                for (int k = j + 1; k < blackboard::Swarm_size; k++) {
                    avgDist = avgDist + Sim1.AllDistances[0][j][k];
                    count++;
                }
            }

            avgDist = avgDist / count;

        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            double pathNum =0;

            for (int j = 0; j < blackboard::Swarm_size; j++)
            {
                for (int k=j+1; k< blackboard::Swarm_size; k++)
                {
                    if ( Sim1.AllDistances[i][j][k] < avgDist)
                    {
                        pathNum = pathNum+1;
                    }

                }
            }

            BetaIndex[i]= pathNum/ double(blackboard::Swarm_size);
        }

        swarm_metrics.push_back(BetaIndex);

    }

    void betaIndex (int swarm_size)
    {

        int count = 0;
        double avgDist = 0.0;

        for (int j = 0; j < swarm_size; j++) {
            for (int k = j + 1; k < swarm_size; k++) {
                avgDist = avgDist + Sim1.AllDistances[0][j][k];
                count++;
            }
        }

        avgDist = avgDist / count;

        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            double pathNum =0;

            for (int j = 0; j < swarm_size; j++)
            {
                for (int k=j+1; k< swarm_size; k++)
                {
                    if ( Sim1.AllDistances[i][j][k] < avgDist)
                    {
                        pathNum = pathNum+1;
                    }

                }
            }

            BetaIndex[i]= pathNum/ double(swarm_size);
        }

        swarm_metrics.push_back(BetaIndex);

    }

    void idle_agents ()
    {

        Idle_agents[0]=0;

        for (long i = 1; i <  blackboard::simulation_timesteps; i++)
        {
            float velx= 0.0;
            float vely= 0.0;
            int idles=0;


            for (int j = 0; j < blackboard::Swarm_size ; j++)
            {
                velx= abs(Sim1.SwarmX[i*blackboard::Swarm_size+j]-Sim1.SwarmX[(i-1)*blackboard::Swarm_size+j]);
                vely= abs(Sim1.SwarmY[i*blackboard::Swarm_size+j]-Sim1.SwarmY[(i-1)*blackboard::Swarm_size+j]);

                if (velx ==0 and vely==0)
                {
                    idles+=1;
                }
            }

            Idle_agents[i]=(idles/float(blackboard::Swarm_size));
        }


    swarm_metrics.push_back(Idle_agents);

    }

    void idle_agents (int swarm_size)
    {

        Idle_agents[0]=0;

        for (long i = 1; i <  blackboard::simulation_timesteps; i++)
        {
            float velx= 0.0;
            float vely= 0.0;
            int idles=0;


            for (int j = 0; j < swarm_size ; j++)
            {
                velx= abs(Sim1.SwarmX[i*swarm_size+j]-Sim1.SwarmX[(i-1)*swarm_size+j]);
                vely= abs(Sim1.SwarmY[i*swarm_size+j]-Sim1.SwarmY[(i-1)*swarm_size+j]);

                if (velx ==0 and vely==0)
                {
                    idles+=1;
                }
            }

            Idle_agents[i]=(idles/float(swarm_size));
        }


        swarm_metrics.push_back(Idle_agents);

    }

    void network_metrics ()
    {
        double avgDist = 0.0;
        int count = 0;

        for (int j = 0; j < blackboard::Swarm_size; j++) {
            for (int k = j + 1; k < blackboard::Swarm_size; k++) {
                avgDist = avgDist + Sim1.AllDistances[0][j][k];
                count++;
            }
        }

        avgDist = avgDist / count;

        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            double pathNum =0;
            double path_length =0.0;
            double clustering_coeff =0.0;


            for (int j = 0; j < blackboard::Swarm_size; j++)
            {
                double local_paths =0.0;
                double local_density=0.0;
                for (int k=j+1; k< blackboard::Swarm_size; k++)
                {
                    if ( Sim1.AllDistances[i][j][k] < avgDist)
                    {
                        pathNum = pathNum+1;
                        path_length = path_length+Sim1.AllDistances[i][j][k];

                    }

                }


            }
            float swarm_area;
            swarm_area=pow((longest_path[i]/2),2)*PI;

            BetaIndex[i]= pathNum/ double(blackboard::Swarm_size);
            swarm_area_density[i]= path_length/ swarm_area;
        }

        swarm_metrics.push_back(BetaIndex);
        swarm_metrics.push_back(swarm_area_density);

    }

    void network_metrics3 ()
    {
        double avgDist = 0.0;
        int count = 0;

        for (int j = 0; j < blackboard::Swarm_size; j++) {
            for (int k = j + 1; k < blackboard::Swarm_size; k++) {
                avgDist = avgDist + Sim1.AllDistances[0][j][k];
                count++;
            }
        }

        avgDist = avgDist / count;

        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            double pathNum =0;
            double path_length =0.0;
            double clustering_coeff =0.0;


            for (int j = 0; j < blackboard::Swarm_size; j++)
            {
                double local_paths =0.0;
                double local_density=0.0;
                for (int k=j+1; k< blackboard::Swarm_size; k++)
                {
                    if ( Sim1.AllDistances[i][j][k] < avgDist)
                    {
                        pathNum = pathNum+1;
                        path_length = path_length+Sim1.AllDistances[i][j][k];

                        if (Sim1.AllDistances[i][j][k] <= blackboard::local_radius && Sim1.AllDistances[i][j][k]>0)
                        {
                            local_paths +=1;
                        }
                    }
                    if (Sim1.AllDistances[i][j][k] <= blackboard::local_radius && Sim1.AllDistances[i][j][k]>0)
                    {
                        local_density +=1;
                    }

                }

                if(local_density>1)
                {
                    clustering_coeff +=((2*local_paths)/(local_density*(local_density-1)));
                }

            }
            float swarm_area;
            swarm_area=pow((longest_path[i]/2),2)*PI;

            BetaIndex[i]= pathNum/ double(blackboard::Swarm_size);
            swarm_area_density[i]= path_length/ swarm_area;
            avg_clustering_coeff[i]= clustering_coeff/ double(blackboard::Swarm_size);
        }

        swarm_metrics.push_back(BetaIndex);
        swarm_metrics.push_back(swarm_area_density);
        swarm_metrics.push_back(avg_clustering_coeff);

    }

    void agent_clustering_coeff (int agent_id)
    {
        double avgDist = 0.0;
        int count = 0;

        for (int j = 0; j < blackboard::Swarm_size; j++) {
            for (int k = j + 1; k < blackboard::Swarm_size; k++) {
                avgDist = avgDist + Sim1.AllDistances[0][j][k];
                count++;
            }
        }

        avgDist = avgDist / count;

        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
                double local_paths =0.0;
                double local_density=0.0;
                for (int k=agent_id+1; k< blackboard::Swarm_size; k++)
                {
                    if ( Sim1.AllDistances[i][agent_id][k] < avgDist)
                    {
                        if (Sim1.AllDistances[i][agent_id][k] <= blackboard::local_radius && Sim1.AllDistances[i][agent_id][k]>0)
                        {
                            local_paths +=1;
                        }
                    }
                    if (Sim1.AllDistances[i][agent_id][k] <= blackboard::local_radius && Sim1.AllDistances[i][agent_id][k]>0)
                    {
                        local_density +=1;
                    }



            }
            if(local_density>1)
            {
                clustering_coeff[i] =((2*local_paths)/(local_density*(local_density-1)));
            }
            else
            {
                clustering_coeff[i] =0;
            }

        }

        swarm_metrics.push_back(clustering_coeff);

    }



    void velocity_SD ()
    {
        float EPSILON = 0.1;


        for (int i = 1; i < blackboard::simulation_timesteps; i++)
        {
            double velx= 0.0;
            double vely= 0.0;

            for (int j = 0; j < blackboard::Swarm_size; j++)
            {
                velx=velx+(Sim1.SwarmX[i*blackboard::Swarm_size+j]-Sim1.SwarmX[(i-1)*blackboard::Swarm_size+j]);
                vely=vely+(Sim1.SwarmY[i*blackboard::Swarm_size+j]-Sim1.SwarmY[(i-1)*blackboard::Swarm_size+j]);
                //VelX[i-1][j]=(Sim1.SwarmX[i][j]-Sim1.SwarmX[i-1][j]);
                //VelY[i-1][j]=(Sim1.SwarmY[i][j]-Sim1.SwarmY[i-1][j]);

            }

            AvgVelX[i-1]=velx/blackboard::Swarm_size;
            AvgVelY[i-1]=vely/blackboard::Swarm_size;
        }


        for (int i =0 ; i <  blackboard::simulation_timesteps; i++) {
            double VelxSum=0.0, VelySum=0.0;

            for (int j = 0; j < blackboard::Swarm_size; j++) {
               // VelxSum +=pow( VelX[i][j]-AvgVelX[i],2);
               // VelySum +=pow(VelY[i][j]- AvgVelY[i],2);

            }

            SDVelX[i]=sqrt(VelxSum/blackboard::Swarm_size);
            SDVelY[i]=sqrt(VelySum/blackboard::Swarm_size);
        }

        swarm_metrics.push_back(AvgVelX);
        swarm_metrics.push_back(AvgVelY);

    }



    void object_metrics ()
    {


        float AvgVelX_temp=0;
        float AvgVelY_temp=0;
        float dist_temp;
        double avgDist = 0.0;
        int count = 0;
        float object_dropped=0.0;
        float object_picked=0.0;
        float pathNum=0;
        float objects_NSD=0;
        vector<int> picked_objects_id;

        for (int a = 0; a < Sim1.controller.blackboard.BT_Env.objects_num; a++) {
            for (int j = 0; j < blackboard::Swarm_size; j++) {
                avgDist = avgDist + sqrt(pow((Sim1.SwarmX[j])  - (Sim1.objects_Data_X[0][a]), 2) + pow((Sim1.SwarmY[j])- (Sim1.objects_Data_Y[0][a]), 2));
                count++;
            }
        }

        avgDist = (avgDist / count);

        for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++) {
            picked_objects_id.push_back(0);
        }

        objects_dropped[0]=(0.0);
        objects_picked[0]=(0.0);
        objects_Beta[0]=0;
        objects_avg_NSD[0]=0;

        for (int j=0; j< blackboard::Swarm_size;j++)
        {
            for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++){
                dist_temp = sqrt(pow((Sim1.SwarmX[j])  - (Sim1.objects_Data_X[0][b]), 2) + pow((Sim1.SwarmY[j])- (Sim1.objects_Data_Y[0][b]), 2));
                if ( dist_temp < avgDist) {
                    objects_Beta[0] = objects_Beta[0] + 1;
                }
            }
        }
        objects_Beta[0]/=float(Sim1.controller.blackboard.BT_Env.objects_num+blackboard::Swarm_size);

        for (int i = 1; i < blackboard::simulation_timesteps; i++)
        {
            object_picked=0.0;
            pathNum=0;
            objects_NSD=0;
            object_dropped=0;
            for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++)
            {

                //Average displacement.
                AvgVelX_temp=Sim1.objects_Data_X[i][b]-Sim1.objects_Data_X[i-1][b];
                AvgVelY_temp=Sim1.objects_Data_Y[i][b]-Sim1.objects_Data_Y[i-1][b];
                //Percentage of moving objects
                if(AvgVelX_temp!=0 or AvgVelY_temp!=0)
                {
                    object_picked+=1;
                } else
                {
                    if(picked_objects_id[b]==1)
                    {
                        object_dropped+=1;
                        picked_objects_id[b]=0;
                    }
                }
                if(AvgVelX_temp!=0 or AvgVelY_temp!=0)
                {
                    picked_objects_id[b]=1;
                }

                //Percentage of moving objects for video
              /*  if(AvgVelX_temp >3 or AvgVelY_temp >3)
                {
                    object_picked+=1;
                }*/

                //Average Net squared displacement (NSD)
                objects_NSD += sqrt(pow((Sim1.objects_Data_X[i][b])  - (Sim1.objects_Data_X[0][b]), 2) + pow((Sim1.objects_Data_Y[i][b])- (Sim1.objects_Data_Y[0][b]), 2));


                //Beta index (objects, agents)
                for (int j=0; j< blackboard::Swarm_size;j++)
                {
                    dist_temp = sqrt(pow((Sim1.SwarmX[i*blackboard::Swarm_size+j])  - (Sim1.objects_Data_X[i][b]), 2) + pow((Sim1.SwarmY[i*blackboard::Swarm_size+j])- (Sim1.objects_Data_Y[i][b]), 2));
                    if ( dist_temp < avgDist) {
                        pathNum = pathNum + 1;
                    }

                }
            }



            objects_picked[i]=(object_picked);
            objects_dropped[i]=(object_dropped);
            objects_Beta[i]=(pathNum/(float(blackboard::Swarm_size+Sim1.controller.blackboard.BT_Env.objects_num)));
            objects_avg_NSD[i]=(objects_NSD/float(Sim1.controller.blackboard.BT_Env.objects_num));
        }


        swarm_metrics.push_back(objects_picked);
        swarm_metrics.push_back(objects_dropped);
        swarm_metrics.push_back(objects_Beta);
        swarm_metrics.push_back(objects_avg_NSD);
    }

    void area_metrics ()
    {
        int area=0;

        float NNdist;
        float FNdist;
        int NN_id;
        float dist_temp;
        double avgDist = 0.0;
        int count = 0;
        float area_density=0;
        float visited_areas=0;
        float acc_visited_areas_sum=0;
        float sumNNdist = 0;
        float FN_path = 1000000;
        float pathNum=0;

        for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++) {
            for (int j = 0; j < blackboard::Swarm_size; j++) {
                avgDist = avgDist + sqrt(pow((Sim1.SwarmX[j])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.SwarmY[j])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                count++;
            }
        }

        avgDist = (avgDist / count);


        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            area_density=0;
            visited_areas=0;
            acc_visited_areas_sum=0;
            sumNNdist = 0;
            FN_path = 1000000;
            pathNum=0;

            for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
            {
                FNdist =0;
                NN_id=-1;
                for (int j=0; j< blackboard::Swarm_size;j++)
                {
                    if ((Sim1.SwarmX[i*blackboard::Swarm_size+j]) >= (Sim1.controller.blackboard.BT_Env.areas_X[a]) &&
                        (Sim1.SwarmX[i*blackboard::Swarm_size+j]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_X[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a]) &&
                        (Sim1.SwarmY[i*blackboard::Swarm_size+j]) >= (Sim1.controller.blackboard.BT_Env.areas_Y[a]) &&
                        (Sim1.SwarmY[i*blackboard::Swarm_size+j]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_Y[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a])) {

                        //Percentage of agents inside the area.
                        area_density += 1;
                        area=1;

                    }
                    dist_temp = sqrt(pow((Sim1.SwarmX[i*blackboard::Swarm_size+j])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.SwarmY[i*blackboard::Swarm_size+j])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                    if (dist_temp > FNdist )
                    {
                        FNdist= dist_temp;
                    }
                    //Beta index (areas, agents)
                    if ( dist_temp < avgDist) {
                        pathNum = pathNum + 1;
                    }
                }
                //sumNNdist =  sumNNdist + NNdist ;
                // Shortest of the longest paths from each area to an agent.
                if (FNdist < FN_path )
                {
                    FN_path=FNdist;
                }
            }
            for (int j=0; j< blackboard::Swarm_size;j++)
            {
                NNdist = 100000;
                for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
                {
                    dist_temp = sqrt(pow((Sim1.SwarmX[i*blackboard::Swarm_size+j])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.SwarmY[i*blackboard::Swarm_size+j])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                    //Average nearest agent distance.
                    if (dist_temp < NNdist )
                    {
                        NNdist= dist_temp;
                    }
                }

                sumNNdist =  sumNNdist + NNdist ;
            }

            agents_inside_area_ratio[i]=(area_density/float(blackboard::Swarm_size));
            areas_avg_NN[i]=(sumNNdist/float(blackboard::Swarm_size));
            areas_avg_FN[i]=(FN_path);
            areas_Beta[i]=(pathNum/float((blackboard::Swarm_size+Sim1.controller.blackboard.BT_Env.areas_num)));

        }
        swarm_metrics.push_back(agents_inside_area_ratio);
        swarm_metrics.push_back(areas_Beta);
        swarm_metrics.push_back(areas_avg_NN);
        swarm_metrics.push_back(areas_avg_FN);
    }

    void object_areas_metrics ()
    {


        float NNdist;
        float NNdist2;
        float FNdist;
        int NN_id;
        int NN_id2;
        float dist_temp;
        float sumNNdist = 0;

        double avgDist = 0.0;
        int count = 0;

        for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++) {
            for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++) {
                avgDist = avgDist + sqrt(pow((Sim1.objects_Data_X[0][b])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.objects_Data_Y[0][b])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                count++;
            }
        }

        avgDist = avgDist / count;





        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            float area_density=0;
            float FN_path = 1000000;
            float pathNum=0;
            float AvgVelX_temp=0;
            float AvgVelY_temp=0;
            sumNNdist = 0;

            for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
            {
                NNdist = 100000;
                FNdist =0;
                NN_id=-1;
                NN_id2=-1;
                for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++)
                {
                    if ((Sim1.objects_Data_X[i][b]) >= (Sim1.controller.blackboard.BT_Env.areas_X[a]) &&
                        (Sim1.objects_Data_X[i][b]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_X[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a]) &&
                        (Sim1.objects_Data_Y[i][b]) >= (Sim1.controller.blackboard.BT_Env.areas_Y[a]) &&
                        (Sim1.objects_Data_Y[i][b]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_Y[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a])) {

                        //Percentage of agents inside the area.
                        area_density += 1;

                    }

                    dist_temp = sqrt(pow((Sim1.objects_Data_X[i][b])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.objects_Data_Y[i][b])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));


                    if (dist_temp > FNdist )
                    {
                        FNdist= dist_temp;
                    }

                    //Beta index (areas, agents)
                    if ( dist_temp < avgDist) {
                        pathNum = pathNum + 1;
                    }

                    //Average nearest agent distance.
                    if (dist_temp < NNdist )
                    {
                        NNdist= dist_temp;
                    }



                }
                // Shortest of the longest paths from each area to an agent.
                if (FNdist < FN_path )
                {
                    FN_path=FNdist;
                }
                sumNNdist =  sumNNdist + NNdist ;



            }


            objects_inside_area_ratio[i]=(area_density/float(Sim1.controller.blackboard.BT_Env.objects_num));
            areas_avg_FN2[i]=(FN_path);
            areas_Beta2[i]=(pathNum/floor((Sim1.controller.blackboard.BT_Env.objects_num+Sim1.controller.blackboard.BT_Env.areas_num)/2));
            areas_avg_NN2[i]=(sumNNdist/float(Sim1.controller.blackboard.BT_Env.areas_num));
        }


        swarm_metrics.push_back(areas_Beta2);
        swarm_metrics.push_back(areas_avg_FN2);
        //swarm_metrics.push_back(areas_avg_NN2);
        swarm_metrics.push_back(objects_inside_area_ratio);

    }

    void object_metrics4 ()
    {


        float AvgVelX=0;
        float AvgVelY=0;
        float center_X=0;
        float center_Y=0;
        float AvgVelX_temp=0;
        float AvgVelY_temp=0;
        float dist_temp;
        double avgDist = 0.0;
        float NO_Dist = 0.0;
        float Avg_NO_Dist = 0.0;
        int count = 0;
        float object_picked=0.0;
        float object_dropped=0.0;
        float pathNum=0;
        float objects_NSD=0;
        float objects_NSD_temp=0;
        vector<int> picked_objects_id;

        for (int a = 0; a < Sim1.controller.blackboard.BT_Env.objects_num; a++) {
            for (int j = a+1; j < Sim1.controller.blackboard.BT_Env.objects_num; j++) {
                avgDist = avgDist + sqrt(pow((Sim1.objects_Data_X[0][j])  - (Sim1.objects_Data_X[0][a]), 2) + pow((Sim1.objects_Data_Y[0][j])- (Sim1.objects_Data_Y[0][a]), 2));
                count++;
            }
        }

        avgDist = avgDist / count;
        for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++) {
            picked_objects_id.push_back(0);
        }
        objects_AvgVelX[0]=(0.0);
        objects_AvgVelY[0]=(0.0);
        objects_picked[0]=(0.0);
        objects_dropped[0]=(0.0);
        objects_Beta[0]=0;
        objects_avg_NSD[0]=0;

        for (int j=0; j< Sim1.controller.blackboard.BT_Env.objects_num;j++)
        {
            for (int b = j+1; b < Sim1.controller.blackboard.BT_Env.objects_num; b++){
                dist_temp = sqrt(pow((Sim1.objects_Data_X[0][j])  - (Sim1.objects_Data_X[0][b]), 2) + pow((Sim1.objects_Data_Y[0][j])- (Sim1.objects_Data_Y[0][b]), 2));
                if ( dist_temp < avgDist) {
                    objects_Beta[0] = objects_Beta[0] + 1;
                }
            }
        }
        objects_Beta[0]/=float(Sim1.controller.blackboard.BT_Env.objects_num);

        for (int i = 1; i < blackboard::simulation_timesteps; i++)
        {
            object_picked=0.0;
            object_dropped=0.0;
            AvgVelX=0;
            AvgVelY=0;
            center_X=0;
            center_Y=0;
            pathNum=0;
            objects_NSD=0;
            objects_NSD_temp=0;
            Avg_NO_Dist = 0.0;
            for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++)
            {

                //Average displacement.
                AvgVelX_temp=Sim1.objects_Data_X[i][b]-Sim1.objects_Data_X[i-1][b];
                AvgVelY_temp=Sim1.objects_Data_Y[i][b]-Sim1.objects_Data_Y[i-1][b];

                //object center of mass
                 center_X+=Sim1.objects_Data_X[i][b];
                 center_Y+=Sim1.objects_Data_Y[i][b];
                //Percentage of moving objects.
                if(AvgVelX_temp!=0 or AvgVelY_temp!=0)
                {
                    object_picked+=1;
                } else
                {
                    if(picked_objects_id[b]==1)
                    {
                        object_dropped+=1;
                        picked_objects_id[b]=0;
                    }
                }
                if(AvgVelX_temp!=0 or AvgVelY_temp!=0)
                {
                    picked_objects_id[b]=1;
                }
                AvgVelX+=AvgVelX_temp;
                AvgVelY+=AvgVelY_temp;

                //Average Net squared displacement (NSD)
                objects_NSD_temp = sqrt(pow((Sim1.objects_Data_X[i][b])  - (Sim1.objects_Data_X[0][b]), 2) + pow((Sim1.objects_Data_Y[i][b])- (Sim1.objects_Data_Y[0][b]), 2));

                if (std::isnan(objects_NSD_temp) || std::isinf(objects_NSD_temp))
                {
                } else
                {
                    objects_NSD+=objects_NSD_temp;
                }

                NO_Dist = 10000;
                //Beta index (objects, agents)
                for (int j=b+1; j< Sim1.controller.blackboard.BT_Env.objects_num;j++)
                {
                    dist_temp = sqrt(pow((Sim1.objects_Data_X[i][j])  - (Sim1.objects_Data_X[i][b]), 2) + pow((Sim1.objects_Data_Y[i][j])- (Sim1.objects_Data_Y[i][b]), 2));
                    if ( dist_temp < avgDist) {
                        pathNum = pathNum + 1;
                    }

                }

                for (int j=0; j< Sim1.controller.blackboard.BT_Env.objects_num;j++)
                {
                    dist_temp = sqrt(pow((Sim1.objects_Data_X[i][j])  - (Sim1.objects_Data_X[i][b]), 2) + pow((Sim1.objects_Data_Y[i][j])- (Sim1.objects_Data_Y[i][b]), 2));

                    if ( dist_temp < NO_Dist and b!=j) {
                        NO_Dist = dist_temp;
                    }

                }

                Avg_NO_Dist+=NO_Dist;
            }


            objects_picked[i]=(object_picked);
            objects_dropped[i]=(object_dropped);
            objects_Beta[i]=(pathNum/float(Sim1.controller.blackboard.BT_Env.objects_num));
            objects_avg_NSD[i]=(objects_NSD/float(Sim1.controller.blackboard.BT_Env.objects_num));
            objects_AvgVelX[i]=(AvgVelX/float(Sim1.controller.blackboard.BT_Env.objects_num));
            objects_AvgVelY[i]=(AvgVelY/float(Sim1.controller.blackboard.BT_Env.objects_num));
            objects_centerMassX[i]=(center_X/float(Sim1.controller.blackboard.BT_Env.objects_num));
            objects_centerMassY[i]=(center_Y/float(Sim1.controller.blackboard.BT_Env.objects_num));
            NNdist[i]=(Avg_NO_Dist/float(Sim1.controller.blackboard.BT_Env.objects_num));

        }

        //swarm_metrics.push_back(objects_centerMassX);
        //swarm_metrics.push_back(objects_centerMassY);
        swarm_metrics.push_back(objects_AvgVelX);
        swarm_metrics.push_back(objects_AvgVelY);
        //swarm_metrics.push_back(objects_AvgVelX);
        //swarm_metrics.push_back(objects_AvgVelY);
        swarm_metrics.push_back(objects_picked);
        //swarm_metrics.push_back(objects_dropped);
        swarm_metrics.push_back(NNdist);
        //swarm_metrics.push_back(objects_Beta);
        //swarm_metrics.push_back(objects_avg_NSD);


    }

    void object_metrics3 ()
    {


        float AvgVelX=0;
        float AvgVelY=0;
        float AvgVelX_temp=0;
        float AvgVelY_temp=0;
        float dist_temp;
        double avgDist = 0.0;
        int count = 0;
        float object_picked=0.0;
        float object_dropped=0.0;
        float pathNum=0;
        float objects_NSD=0;
        float objects_NSD_temp=0;
        vector<int> picked_objects_id;

        for (int a = 0; a < Sim1.controller.blackboard.BT_Env.objects_num; a++) {
            for (int j = a+1; j < Sim1.controller.blackboard.BT_Env.objects_num; j++) {
                avgDist = avgDist + sqrt(pow((Sim1.objects_Data_X[0][j])  - (Sim1.objects_Data_X[0][a]), 2) + pow((Sim1.objects_Data_Y[0][j])- (Sim1.objects_Data_Y[0][a]), 2));
                count++;
            }
        }

        avgDist = avgDist / count;
        for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++) {
            picked_objects_id.push_back(0);
        }
        objects_AvgVelX[0]=(0.0);
        objects_AvgVelY[0]=(0.0);
        objects_picked[0]=(0.0);
        objects_dropped[0]=(0.0);
        objects_Beta[0]=0;
        objects_avg_NSD[0]=0;

        for (int j=0; j< Sim1.controller.blackboard.BT_Env.objects_num;j++)
        {
            for (int b = j+1; b < Sim1.controller.blackboard.BT_Env.objects_num; b++){
                dist_temp = sqrt(pow((Sim1.objects_Data_X[0][j])  - (Sim1.objects_Data_X[0][b]), 2) + pow((Sim1.objects_Data_Y[0][j])- (Sim1.objects_Data_Y[0][b]), 2));
                if ( dist_temp < avgDist) {
                    objects_Beta[0] = objects_Beta[0] + 1;
                }
            }
        }
        objects_Beta[0]/=float(Sim1.controller.blackboard.BT_Env.objects_num);

        for (int i = 1; i < blackboard::simulation_timesteps; i++)
        {
            object_picked=0.0;
            object_dropped=0.0;
            AvgVelX=0;
            AvgVelY=0;
            pathNum=0;
            objects_NSD=0;
            objects_NSD_temp=0;
            for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++)
            {

                //Average displacement.
                AvgVelX_temp=Sim1.objects_Data_X[i][b]-Sim1.objects_Data_X[i-1][b];
                AvgVelY_temp=Sim1.objects_Data_Y[i][b]-Sim1.objects_Data_Y[i-1][b];
                //Percentage of moving objects.
                if(AvgVelX_temp!=0 or AvgVelY_temp!=0)
                {
                    object_picked+=1;
                } else
                {
                    if(picked_objects_id[b]==1)
                    {
                        object_dropped+=1;
                        picked_objects_id[b]=0;
                    }
                }
                if(AvgVelX_temp!=0 or AvgVelY_temp!=0)
                {
                    picked_objects_id[b]=1;
                }
                AvgVelX+=AvgVelX_temp;
                AvgVelY+=AvgVelY_temp;

                //Average Net squared displacement (NSD)
                objects_NSD_temp = sqrt(pow((Sim1.objects_Data_X[i][b])  - (Sim1.objects_Data_X[0][b]), 2) + pow((Sim1.objects_Data_Y[i][b])- (Sim1.objects_Data_Y[0][b]), 2));

                if (std::isnan(objects_NSD_temp) || std::isinf(objects_NSD_temp))
                {
                } else
                {
                    objects_NSD+=objects_NSD_temp;
                }


                //Beta index (objects, agents)
                for (int j=b+1; j< Sim1.controller.blackboard.BT_Env.objects_num;j++)
                {
                    dist_temp = sqrt(pow((Sim1.objects_Data_X[i][j])  - (Sim1.objects_Data_X[i][b]), 2) + pow((Sim1.objects_Data_Y[i][j])- (Sim1.objects_Data_Y[i][b]), 2));
                    if ( dist_temp < avgDist) {
                        pathNum = pathNum + 1;
                    }

                }
            }


            objects_picked[i]=(object_picked);
            objects_dropped[i]=(object_dropped);
            objects_Beta[i]=(pathNum/float(Sim1.controller.blackboard.BT_Env.objects_num));
            objects_avg_NSD[i]=(objects_NSD/float(Sim1.controller.blackboard.BT_Env.objects_num));
            objects_AvgVelX[i]=(AvgVelX/float(Sim1.controller.blackboard.BT_Env.objects_num));
            objects_AvgVelY[i]=(AvgVelY/float(Sim1.controller.blackboard.BT_Env.objects_num));


        }

        swarm_metrics.push_back(objects_AvgVelX);
        swarm_metrics.push_back(objects_AvgVelY);
        swarm_metrics.push_back(objects_picked);
        //swarm_metrics.push_back(objects_dropped);
        swarm_metrics.push_back(objects_Beta);
        swarm_metrics.push_back(objects_avg_NSD);


    }

    void object_metrics3 (int swarm_size)
    {


        float AvgVelX=0;
        float AvgVelY=0;
        float AvgVelX_temp=0;
        float AvgVelY_temp=0;
        float dist_temp;
        double avgDist = 0.0;
        int count = 0;
        float object_picked=0.0;
        float object_dropped=0.0;
        float pathNum=0;
        float objects_NSD=0;
        float objects_NSD_temp=0;
        vector<int> picked_objects_id;

        for (int a = 0; a < Sim1.controller.blackboard.BT_Env.objects_num; a++) {
            for (int j = a+1; j < Sim1.controller.blackboard.BT_Env.objects_num; j++) {
                avgDist = avgDist + sqrt(pow((Sim1.objects_Data_X[0][j])  - (Sim1.objects_Data_X[0][a]), 2) + pow((Sim1.objects_Data_Y[0][j])- (Sim1.objects_Data_Y[0][a]), 2));
                count++;
            }
        }

        avgDist = avgDist / count;
        for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++) {
            picked_objects_id.push_back(0);
        }
        objects_AvgVelX[0]=(0.0);
        objects_AvgVelY[0]=(0.0);
        objects_picked[0]=(0.0);
        objects_dropped[0]=(0.0);
        objects_Beta[0]=0;
        objects_avg_NSD[0]=0;

        for (int j=0; j< Sim1.controller.blackboard.BT_Env.objects_num;j++)
        {
            for (int b = j+1; b < Sim1.controller.blackboard.BT_Env.objects_num; b++){
                dist_temp = sqrt(pow((Sim1.objects_Data_X[0][j])  - (Sim1.objects_Data_X[0][b]), 2) + pow((Sim1.objects_Data_Y[0][j])- (Sim1.objects_Data_Y[0][b]), 2));
                if ( dist_temp < avgDist) {
                    objects_Beta[0] = objects_Beta[0] + 1;
                }
            }
        }
        objects_Beta[0]/=float(Sim1.controller.blackboard.BT_Env.objects_num);

        for (int i = 1; i < blackboard::simulation_timesteps; i++)
        {
            object_picked=0.0;
            object_dropped=0.0;
            AvgVelX=0;
            AvgVelY=0;
            pathNum=0;
            objects_NSD=0;
            objects_NSD_temp=0;
            for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++)
            {

                //Average displacement.
                AvgVelX_temp=Sim1.objects_Data_X[i][b]-Sim1.objects_Data_X[i-1][b];
                AvgVelY_temp=Sim1.objects_Data_Y[i][b]-Sim1.objects_Data_Y[i-1][b];
                //Percentage of moving objects.
                if(AvgVelX_temp!=0 or AvgVelY_temp!=0)
                {
                    object_picked+=1;
                } else
                {
                    if(picked_objects_id[b]==1)
                    {
                        object_dropped+=1;
                        picked_objects_id[b]=0;
                    }
                }
                if(AvgVelX_temp!=0 or AvgVelY_temp!=0)
                {
                    picked_objects_id[b]=1;
                }
                AvgVelX+=AvgVelX_temp;
                AvgVelY+=AvgVelY_temp;

                //Average Net squared displacement (NSD)
                objects_NSD_temp = sqrt(pow((Sim1.objects_Data_X[i][b])  - (Sim1.objects_Data_X[0][b]), 2) + pow((Sim1.objects_Data_Y[i][b])- (Sim1.objects_Data_Y[0][b]), 2));

                if (std::isnan(objects_NSD_temp) || std::isinf(objects_NSD_temp))
                {
                } else
                {
                    objects_NSD+=objects_NSD_temp;
                }


                //Beta index (objects, agents)
                for (int j=b+1; j< Sim1.controller.blackboard.BT_Env.objects_num;j++)
                {
                    dist_temp = sqrt(pow((Sim1.objects_Data_X[i][j])  - (Sim1.objects_Data_X[i][b]), 2) + pow((Sim1.objects_Data_Y[i][j])- (Sim1.objects_Data_Y[i][b]), 2));
                    if ( dist_temp < avgDist) {
                        pathNum = pathNum + 1;
                    }

                }
            }


            objects_picked[i]=(object_picked);
            objects_dropped[i]=(object_dropped);
            objects_Beta[i]=(pathNum/float(Sim1.controller.blackboard.BT_Env.objects_num));
            objects_avg_NSD[i]=(objects_NSD/float(Sim1.controller.blackboard.BT_Env.objects_num));
            objects_AvgVelX[i]=(AvgVelX/float(Sim1.controller.blackboard.BT_Env.objects_num));
            objects_AvgVelY[i]=(AvgVelY/float(Sim1.controller.blackboard.BT_Env.objects_num));


        }

        swarm_metrics.push_back(objects_AvgVelX);
        swarm_metrics.push_back(objects_AvgVelY);
        swarm_metrics.push_back(objects_picked);
        //swarm_metrics.push_back(objects_dropped);
        swarm_metrics.push_back(objects_Beta);
        swarm_metrics.push_back(objects_avg_NSD);


    }

    void object_metrics (int swarm_size)
    {


        float AvgVelX_temp=0;
        float AvgVelY_temp=0;

        float dist_temp;
        double avgDist = 0.0;
        int count = 0;

        for (int a = 0; a < Sim1.controller.blackboard.BT_Env.objects_num; a++) {
            for (int j = 0; j < swarm_size; j++) {
                avgDist = avgDist + sqrt(pow((Sim1.SwarmX[j])  - (Sim1.objects_Data_X[0][a]), 2) + pow((Sim1.SwarmY[j])- (Sim1.objects_Data_Y[0][a]), 2));
                count++;
            }
        }

        avgDist = avgDist / count;
        objects_agents_avg_distance=avgDist;

        objects_picked[0]=(0.0);
        objects_Beta[0]=0;
        objects_avg_NSD[0]=0;

        for (int j=0; j< swarm_size;j++)
        {
            for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++){
                dist_temp = sqrt(pow((Sim1.SwarmX[j])  - (Sim1.objects_Data_X[0][b]), 2) + pow((Sim1.SwarmY[j])- (Sim1.objects_Data_Y[0][b]), 2));
                if ( dist_temp < avgDist) {
                    objects_Beta[0] = objects_Beta[0] + 1;
                }
            }
        }
        objects_Beta[0]/=float(Sim1.controller.blackboard.BT_Env.objects_num+swarm_size);

        for (int i = 1; i < blackboard::simulation_timesteps; i++)
        {
            float object_picked=0.0;

            float pathNum=0;
            float objects_NSD=0;
            for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++)
            {

                //Average displacement.
                AvgVelX_temp=Sim1.objects_Data_X[i][b]-Sim1.objects_Data_X[i-1][b];
                AvgVelY_temp=Sim1.objects_Data_Y[i][b]-Sim1.objects_Data_Y[i-1][b];
                //Percentage of moving objects
                /*if(AvgVelX_temp!=0 or AvgVelY_temp!=0)
                {
                    object_picked+=1;
                }*/

                //Percentage of moving objects for video
                if(AvgVelX_temp >3 or AvgVelY_temp >3)
                {
                    object_picked+=1;
                }

                //Average Net squared displacement (NSD)
                objects_NSD += sqrt(pow((Sim1.objects_Data_X[i][b])  - (Sim1.objects_Data_X[0][b]), 2) + pow((Sim1.objects_Data_Y[i][b])- (Sim1.objects_Data_Y[0][b]), 2));


                //Beta index (objects, agents)
                for (int j=0; j< swarm_size;j++)
                {
                    dist_temp = sqrt(pow((Sim1.SwarmX[i*swarm_size+j])  - (Sim1.objects_Data_X[i][b]), 2) + pow((Sim1.SwarmY[i*swarm_size+j])- (Sim1.objects_Data_Y[i][b]), 2));
                    if ( dist_temp < avgDist) {
                        pathNum = pathNum + 1;
                    }

                }
            }



            objects_picked[i]=(object_picked);
            objects_Beta[i]=(pathNum/float(Sim1.controller.blackboard.BT_Env.objects_num+swarm_size));
            objects_avg_NSD[i]=(objects_NSD/float(Sim1.controller.blackboard.BT_Env.objects_num));
        }


        swarm_metrics.push_back(objects_picked);
        swarm_metrics.push_back(objects_Beta);
        swarm_metrics.push_back(objects_avg_NSD);
        //swarm_metrics.push_back(objects_picked);
    }

    void area_metrics (int swarm_size)
    {
        int area=0;

        float NNdist;
        float FNdist;
        int NN_id;
        float dist_temp;

        double avgDist = 0.0;
        int count = 0;

        for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++) {
            for (int j = 0; j < swarm_size; j++) {
                avgDist = avgDist + sqrt(pow((Sim1.SwarmX[j])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.SwarmY[j])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                count++;
            }
        }

        avgDist = avgDist / count;

        areas_agents_avg_distance=avgDist;



        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            float area_density=0;
            float visited_areas=0;
            float acc_visited_areas_sum=0;
            float sumNNdist = 0;
            float FN_path = 1000000;
            float pathNum=0;

            for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
            {
                NNdist = 100000;
                FNdist =0;
                NN_id=-1;
                for (int j=0; j< swarm_size;j++)
                {
                    if ((Sim1.SwarmX[i*swarm_size+j]) >= (Sim1.controller.blackboard.BT_Env.areas_X[a]) &&
                        (Sim1.SwarmX[i*swarm_size+j]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_X[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a]) &&
                        (Sim1.SwarmY[i*swarm_size+j]) >= (Sim1.controller.blackboard.BT_Env.areas_Y[a]) &&
                        (Sim1.SwarmY[i*swarm_size+j]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_Y[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a])) {

                        //Percentage of agents inside the area.
                        area_density += 1;
                        area=1;

                    }
                    dist_temp = sqrt(pow((Sim1.SwarmX[i*swarm_size+j])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.SwarmY[i*swarm_size+j])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                    //Average nearest agent distance.
                    if (dist_temp < NNdist )
                    {
                        NNdist= dist_temp;
                        NN_id=j;
                    }
                    if (dist_temp > FNdist )
                    {
                        FNdist= dist_temp;
                    }
                    //Beta index (areas, agents)
                    if ( dist_temp < avgDist) {
                        pathNum = pathNum + 1;
                    }
                }
                //sumNNdist =  sumNNdist + NNdist ;
                // Shortest of the longest paths from each area to an agent.
                if (FNdist < FN_path )
                {
                    FN_path=FNdist;
                }
            }
            for (int j=0; j< swarm_size;j++)
            {
                NNdist = 100000;
                for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
                {
                    dist_temp = sqrt(pow((Sim1.SwarmX[i*swarm_size+j])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.SwarmY[i*swarm_size+j])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                    //Average nearest agent distance.
                    if (dist_temp < NNdist )
                    {
                        NNdist= dist_temp;
                    }
                }

                sumNNdist =  sumNNdist + NNdist ;
            }

            agents_inside_area_ratio[i]=(area_density/float(swarm_size));
            areas_avg_NN[i]=(sumNNdist/float(swarm_size));
            areas_avg_FN[i]=(FN_path);
            areas_Beta[i]=(pathNum/float(Sim1.controller.blackboard.BT_Env.areas_num+swarm_size));
        }
        swarm_metrics.push_back(agents_inside_area_ratio);
        swarm_metrics.push_back(areas_Beta);
        swarm_metrics.push_back(areas_avg_NN);
        swarm_metrics.push_back(areas_avg_FN);
    }

    void object_areas_metrics (int swarm_size)
    {


        float NNdist;
        float NNdist2;
        float FNdist;
        int NN_id;
        int NN_id2;
        float dist_temp;

        double avgDist = 0.0;
        int count = 0;

        for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++) {
            for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++) {
                avgDist = avgDist + sqrt(pow((Sim1.objects_Data_X[0][b])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.objects_Data_Y[0][b])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                count++;
            }
        }

        avgDist = avgDist / count;





        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            float area_density=0;
            float FN_path = 1000000;
            float pathNum=0;
            float AvgVelX_temp=0;
            float AvgVelY_temp=0;


            for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
            {
                NNdist = 100000;
                NNdist2 = 100000;
                FNdist =0;
                NN_id=-1;
                NN_id2=-1;
                for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++)
                {
                    if ((Sim1.objects_Data_X[i][b]) >= (Sim1.controller.blackboard.BT_Env.areas_X[a]) &&
                        (Sim1.objects_Data_X[i][b]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_X[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a]) &&
                        (Sim1.objects_Data_Y[i][b]) >= (Sim1.controller.blackboard.BT_Env.areas_Y[a]) &&
                        (Sim1.objects_Data_Y[i][b]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_Y[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a])) {

                        //Percentage of agents inside the area.
                        area_density += 1;

                    }

                    dist_temp = sqrt(pow((Sim1.objects_Data_X[i][b])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.objects_Data_Y[i][b])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));


                    if (dist_temp > FNdist )
                    {
                        FNdist= dist_temp;
                    }

                    //Beta index (areas, agents)
                    if ( dist_temp < avgDist) {
                        pathNum = pathNum + 1;
                    }



                }
                //sumNNdist =  sumNNdist + NNdist ;
                // Shortest of the longest paths from each area to an agent.
                if (FNdist < FN_path )
                {
                    FN_path=FNdist;
                }



            }

            float areas_objects_path_ratio=0;
            for (int j=0; j< swarm_size;j++)
            {
                float areas_pathNum=0;
                float objects_pathNum=0;
                for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
                {
                    dist_temp = sqrt(pow((Sim1.SwarmX[i*swarm_size+j])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.SwarmY[i*swarm_size+j])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                    //Average nearest agent distance.

                    //Beta index (areas, agents)
                    if ( dist_temp < areas_agents_avg_distance) {
                        areas_pathNum = areas_pathNum + 1;
                    }

                }
                for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++)
                {
                    dist_temp = sqrt(pow((Sim1.SwarmX[i*swarm_size+j])  - (Sim1.objects_Data_X[i][b]), 2) + pow((Sim1.SwarmY[i*swarm_size+j])- (Sim1.objects_Data_Y[i][b]), 2));
                    if ( dist_temp < objects_agents_avg_distance) {
                        objects_pathNum = objects_pathNum + 1;
                    }
                }

                if (objects_pathNum>0)
                {
                    areas_objects_path_ratio+=(areas_pathNum/objects_pathNum);
                }

            }


            objects_inside_area_ratio[i]=(area_density/float(Sim1.controller.blackboard.BT_Env.objects_num));
            areas_avg_FN2[i]=(FN_path);
            areas_Beta2[i]=(pathNum/float(Sim1.controller.blackboard.BT_Env.areas_num+Sim1.controller.blackboard.BT_Env.objects_num));
            objects_paths_ratios[i]=(areas_objects_path_ratio);



        }


        swarm_metrics.push_back(areas_Beta2);
        swarm_metrics.push_back(areas_avg_FN2);
        swarm_metrics.push_back(objects_paths_ratios);
        swarm_metrics.push_back(objects_inside_area_ratio);

    }


    void object_areas_metrics2 ()
    {


        float NNdist;
        float NNdist2;
        float FNdist;
        int NN_id;
        int NN_id2;
        float dist_temp;

        double avgDist = 0.0;
        int count = 0;

        for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++) {
            for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++) {
                avgDist = avgDist + sqrt(pow((Sim1.objects_Data_X[0][b])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.objects_Data_Y[0][b])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                count++;
            }
        }

        avgDist = avgDist / count;





        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            float area_density=0;
            float sumNNdist = 0;
            float FN_path = 1000000;
            float NN_VelX = 0;
            float NN_VelY = 0;
            float pathNum=0;
            float AvgVelX_temp=0;
            float AvgVelY_temp=0;


            for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
            {
                NNdist = 100000;
                NNdist2 = 100000;
                FNdist =0;
                NN_id=-1;
                NN_id2=-1;
                for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++)
                {
                    if ((Sim1.objects_Data_X[i][b]) >= (Sim1.controller.blackboard.BT_Env.areas_X[a]) &&
                        (Sim1.objects_Data_X[i][b]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_X[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a]) &&
                        (Sim1.objects_Data_Y[i][b]) >= (Sim1.controller.blackboard.BT_Env.areas_Y[a]) &&
                        (Sim1.objects_Data_Y[i][b]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_Y[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a])) {

                        //Percentage of agents inside the area.
                        area_density += 1;

                    }
                    dist_temp = sqrt(pow((Sim1.objects_Data_X[i][b])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.objects_Data_Y[i][b])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                    //Average nearest agent distance.
                    if (dist_temp < NNdist )
                    {
                        NNdist= dist_temp;
                        NN_id=b;
                    }

                    if (dist_temp > FNdist )
                    {
                        FNdist= dist_temp;
                    }

                    //Beta index (areas, agents)
                    if ( dist_temp < avgDist) {
                        pathNum = pathNum + 1;
                    }



                }
                //sumNNdist =  sumNNdist + NNdist ;
                // Shortest of the longest paths from each area to an agent.
                if (FNdist < FN_path )
                {
                    FN_path=FNdist;
                }

                //Nearest agent displacement.
                if ( i >0)
                {
                    NN_VelX+= (Sim1.objects_Data_X[i][NN_id]-Sim1.objects_Data_X[(i-1)][NN_id]);
                    NN_VelY+= (Sim1.objects_Data_Y[i][NN_id]-Sim1.objects_Data_Y[(i-1)][NN_id]);

                }


            }

            for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++)
            {
                NNdist = 100000;

                for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
                {

                    dist_temp = sqrt(pow((Sim1.objects_Data_X[i][b])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.objects_Data_Y[i][b])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                    //Average nearest agent distance.
                    if (dist_temp < NNdist )
                    {
                        NNdist= dist_temp;
                    }


                }
                sumNNdist =  sumNNdist + NNdist ;

            }

            float areas_objects_path_ratio=0;
            for (int j=0; j< blackboard::Swarm_size;j++)
            {
                float areas_pathNum=0;
                float objects_pathNum=0;
                for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
                {
                    dist_temp = sqrt(pow((Sim1.SwarmX[i*blackboard::Swarm_size+j])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.SwarmY[i*blackboard::Swarm_size+j])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                    //Average nearest agent distance.

                    //Beta index (areas, agents)
                    if ( dist_temp < areas_agents_avg_distance) {
                        areas_pathNum = areas_pathNum + 1;
                    }

                }
                for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++)
                {
                    dist_temp = sqrt(pow((Sim1.SwarmX[i*blackboard::Swarm_size+j])  - (Sim1.objects_Data_X[i][b]), 2) + pow((Sim1.SwarmY[i*blackboard::Swarm_size+j])- (Sim1.objects_Data_Y[i][b]), 2));
                    if ( dist_temp < objects_agents_avg_distance) {
                        objects_pathNum = objects_pathNum + 1;
                    }
                }

                if (objects_pathNum>0)
                {
                    areas_objects_path_ratio+=(areas_pathNum/objects_pathNum);
                }

            }


            objects_inside_area_ratio[i]=(area_density/float(Sim1.controller.blackboard.BT_Env.objects_num));
            areas_avg_NN2[i]=(sumNNdist/float(Sim1.controller.blackboard.BT_Env.objects_num));
            areas_avg_FN2[i]=(FN_path);
            areas_Beta2[i]=(pathNum/float(Sim1.controller.blackboard.BT_Env.areas_num+Sim1.controller.blackboard.BT_Env.objects_num));
            objects_paths_ratios[i]=(areas_objects_path_ratio);
            if ( i >0)
            {
                avg_NN_VelX2[i]=(NN_VelX/float(Sim1.controller.blackboard.BT_Env.areas_num));
                avg_NN_VelY2[i]=(NN_VelY/float(Sim1.controller.blackboard.BT_Env.areas_num));
            }


        }

        swarm_metrics.push_back(objects_inside_area_ratio);
        swarm_metrics.push_back(avg_NN_VelX2);
        swarm_metrics.push_back(avg_NN_VelY2);
        swarm_metrics.push_back(areas_Beta2);
        swarm_metrics.push_back(areas_avg_NN2);
        swarm_metrics.push_back(areas_avg_FN2);
        swarm_metrics.push_back(objects_paths_ratios);

    }


    void area_metrics2 ()
    {
        int area=0;

        float NNdist;
        float FNdist;
        int NN_id;
        float dist_temp;

        double avgDist = 0.0;
        int count = 0;

        for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++) {
            for (int j = 0; j < blackboard::Swarm_size; j++) {
                avgDist = avgDist + sqrt(pow((Sim1.SwarmX[j])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.SwarmY[j])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                count++;
            }
        }

        avgDist = avgDist / count;

        areas_agents_avg_distance=avgDist;



        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            float area_density=0;
            float visited_areas=0;
            float acc_visited_areas_sum=0;
            float sumNNdist = 0;
            float FN_path = 1000000;
            float NN_VelX = 0;
            float NN_VelY = 0;
            float pathNum=0;


            for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
            {
                NNdist = 100000;
                FNdist =0;
                NN_id=-1;
                for (int j=0; j< blackboard::Swarm_size;j++)
                {
                    if ((Sim1.SwarmX[i*blackboard::Swarm_size+j]) >= (Sim1.controller.blackboard.BT_Env.areas_X[a]) &&
                        (Sim1.SwarmX[i*blackboard::Swarm_size+j]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_X[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a]) &&
                        (Sim1.SwarmY[i*blackboard::Swarm_size+j]) >= (Sim1.controller.blackboard.BT_Env.areas_Y[a]) &&
                        (Sim1.SwarmY[i*blackboard::Swarm_size+j]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_Y[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a])) {

                        //Percentage of agents inside the area.
                        area_density += 1;
                        area=1;

                    }
                    dist_temp = sqrt(pow((Sim1.SwarmX[i*blackboard::Swarm_size+j])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.SwarmY[i*blackboard::Swarm_size+j])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                    //Average nearest agent distance.
                    if (dist_temp < NNdist )
                    {
                        NNdist= dist_temp;
                        NN_id=j;
                    }

                    if (dist_temp > FNdist )
                    {
                        FNdist= dist_temp;
                    }

                    //Beta index (areas, agents)
                    if ( dist_temp < avgDist) {
                        pathNum = pathNum + 1;
                    }



                }
                //sumNNdist =  sumNNdist + NNdist ;
                // Shortest of the longest paths from each area to an agent.
                if (FNdist < FN_path )
                {
                    FN_path=FNdist;
                }

                //Nearest agent displacement.
                if ( i >0)
                {
                    NN_VelX+= (Sim1.SwarmX[i*blackboard::Swarm_size+NN_id]-Sim1.SwarmX[(i-1)*blackboard::Swarm_size+NN_id]);
                    NN_VelY+= (Sim1.SwarmY[i*blackboard::Swarm_size+NN_id]-Sim1.SwarmY[(i-1)*blackboard::Swarm_size+NN_id]);

                }


            }

            for (int j=0; j< blackboard::Swarm_size;j++)
            {
                NNdist = 100000;
                for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
                {
                    dist_temp = sqrt(pow((Sim1.SwarmX[i*blackboard::Swarm_size+j])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.SwarmY[i*blackboard::Swarm_size+j])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                    //Average nearest agent distance.
                    if (dist_temp < NNdist )
                    {
                        NNdist= dist_temp;
                    }
                }

                sumNNdist =  sumNNdist + NNdist ;
            }



            agents_inside_area_ratio[i]=(area_density/float(blackboard::Swarm_size));
            areas_avg_NN[i]=(sumNNdist/float(blackboard::Swarm_size));
            areas_avg_FN[i]=(FN_path);
            areas_Beta[i]=(pathNum/float(Sim1.controller.blackboard.BT_Env.areas_num+blackboard::Swarm_size));
            if ( i >0)
            {
                avg_NN_VelX[i]=(NN_VelX/float(Sim1.controller.blackboard.BT_Env.areas_num));
                avg_NN_VelY[i]=(NN_VelY/float(Sim1.controller.blackboard.BT_Env.areas_num));
            }


        }

        swarm_metrics.push_back(agents_inside_area_ratio);
        swarm_metrics.push_back(avg_NN_VelX);
        swarm_metrics.push_back(avg_NN_VelY);
        swarm_metrics.push_back(areas_Beta);
        swarm_metrics.push_back(areas_avg_NN);
        swarm_metrics.push_back(areas_avg_FN);

    }
    void local_env_metrics (int agents_num)
    {
        for (int i=0;i< agents_num;i+=1) {
            int agent_id = rand() % blackboard::Swarm_size;

        }
    }

    void object_areas_metrics3 ()
    {


        float NNdist;
        float NNdist2;
        float FNdist;
        int NN_id;
        int NN_id2;
        float dist_temp;

        double avgDist = 0.0;
        int count = 0;

        for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++) {
            for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++) {
                avgDist = avgDist + sqrt(pow((Sim1.objects_Data_X[0][b])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.objects_Data_Y[0][b])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                count++;
            }
        }

        avgDist = avgDist / count;





        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            float area_density=0;
            float FN_path = 1000000;
            float pathNum=0;
            float AvgVelX_temp=0;
            float AvgVelY_temp=0;


            for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
            {
                NNdist = 100000;
                NNdist2 = 100000;
                FNdist =0;
                NN_id=-1;
                NN_id2=-1;
                for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++)
                {
                    if ((Sim1.objects_Data_X[i][b]) >= (Sim1.controller.blackboard.BT_Env.areas_X[a]) &&
                        (Sim1.objects_Data_X[i][b]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_X[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a]) &&
                        (Sim1.objects_Data_Y[i][b]) >= (Sim1.controller.blackboard.BT_Env.areas_Y[a]) &&
                        (Sim1.objects_Data_Y[i][b]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_Y[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a])) {

                        //Percentage of agents inside the area.
                        area_density += 1;

                    }

                    dist_temp = sqrt(pow((Sim1.objects_Data_X[i][b])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.objects_Data_Y[i][b])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));


                    if (dist_temp > FNdist )
                    {
                        FNdist= dist_temp;
                    }

                    //Beta index (areas, agents)
                    if ( dist_temp < avgDist) {
                        pathNum = pathNum + 1;
                    }



                }
                //sumNNdist =  sumNNdist + NNdist ;
                // Shortest of the longest paths from each area to an agent.
                if (FNdist < FN_path )
                {
                    FN_path=FNdist;
                }



            }

            float areas_objects_path_ratio=0;
            for (int j=0; j< blackboard::Swarm_size;j++)
            {
                float areas_pathNum=0;
                float objects_pathNum=0;
                for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
                {
                    dist_temp = sqrt(pow((Sim1.SwarmX[i*blackboard::Swarm_size+j])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.SwarmY[i*blackboard::Swarm_size+j])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                    //Average nearest agent distance.

                    //Beta index (areas, agents)
                    if ( dist_temp < areas_agents_avg_distance) {
                        areas_pathNum = areas_pathNum + 1;
                    }

                }
                for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++)
                {
                    dist_temp = sqrt(pow((Sim1.SwarmX[i*blackboard::Swarm_size+j])  - (Sim1.objects_Data_X[i][b]), 2) + pow((Sim1.SwarmY[i*blackboard::Swarm_size+j])- (Sim1.objects_Data_Y[i][b]), 2));
                    if ( dist_temp < objects_agents_avg_distance) {
                        objects_pathNum = objects_pathNum + 1;
                    }
                }

                if (objects_pathNum>0)
                {
                    areas_objects_path_ratio+=(areas_pathNum/objects_pathNum);
                }

            }


            objects_inside_area_ratio[i]=(area_density/float(Sim1.controller.blackboard.BT_Env.objects_num));
            areas_avg_FN2[i]=(FN_path);
            areas_Beta2[i]=(pathNum/floor((Sim1.controller.blackboard.BT_Env.objects_num+Sim1.controller.blackboard.BT_Env.areas_num)/2));
            //objects_paths_ratios[i]=(areas_objects_path_ratio);



        }


        swarm_metrics.push_back(areas_Beta2);
        swarm_metrics.push_back(areas_avg_FN2);
        swarm_metrics.push_back(objects_inside_area_ratio);

    }


    void object_metrics2 ()
    {
        float AvgVelX=0;
        float AvgVelY=0;
        float AvgVelX_temp=0;
        float AvgVelY_temp=0;
        float objects_center_x=0;
        float objects_center_y=0;
        for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++) {
            objects_center_x += Sim1.objects_Data_X[0][b];
            objects_center_y += Sim1.objects_Data_Y[0][b];
        }
        objects_centerMassX[0]=(objects_center_x/float(Sim1.controller.blackboard.BT_Env.objects_num));
        objects_centerMassY[0]=(objects_center_y/float(Sim1.controller.blackboard.BT_Env.objects_num));
        objects_AvgVelX[0]=(0.0);
        objects_AvgVelY[0]=(0.0);
        objects_picked[0]=(0.0);

        for (int i = 1; i < blackboard::simulation_timesteps; i++)
        {
            float object_picked=0.0;
            objects_center_x=0;
            objects_center_y=0;
            AvgVelX=0;
            AvgVelY=0;
            for (int b = 0; b < Sim1.controller.blackboard.BT_Env.objects_num; b++)
            {
                objects_center_x+=Sim1.objects_Data_X[i][b];
                objects_center_y+=Sim1.objects_Data_Y[i][b];
                AvgVelX_temp=Sim1.objects_Data_X[i][b]-Sim1.objects_Data_X[i-1][b];
                AvgVelY_temp=Sim1.objects_Data_Y[i][b]-Sim1.objects_Data_Y[i-1][b];
                if(AvgVelX_temp!=0 or AvgVelY_temp!=0)
                {
                    object_picked+=1;
                }
                AvgVelX+=AvgVelX_temp;
                AvgVelY+=AvgVelY_temp;
            }

            objects_centerMassX[i]=(objects_center_x/float(Sim1.controller.blackboard.BT_Env.objects_num));
            objects_centerMassY[i]=(objects_center_y/float(Sim1.controller.blackboard.BT_Env.objects_num));
            objects_AvgVelX[i]=(AvgVelX/float(Sim1.controller.blackboard.BT_Env.objects_num));
            objects_AvgVelY[i]=(AvgVelY/float(Sim1.controller.blackboard.BT_Env.objects_num));
            objects_picked[i]=(object_picked);
        }
        swarm_metrics.push_back(objects_centerMassX);
        swarm_metrics.push_back(objects_centerMassY);
        swarm_metrics.push_back(objects_AvgVelX);
        swarm_metrics.push_back(objects_AvgVelY);
        swarm_metrics.push_back(objects_picked);
        //swarm_metrics.push_back(objects_picked);
    }

    void area_metrics3 ()
    {
        int area=0;
        float areas_center_x=0;
        float areas_center_y=0;
        vector<int> acc_visited_areas={};
        for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
        {
            acc_visited_areas.push_back(0);
            areas_center_x+=(Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2));
            areas_center_y+=(Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2));
        }
        areas_center_x/=float(Sim1.controller.blackboard.BT_Env.areas_num);
        areas_center_y/=float(Sim1.controller.blackboard.BT_Env.areas_num);

        float NNdist;
        float FNdist;
        float dist_temp;

        for (int i = 0; i < blackboard::simulation_timesteps; i++)
        {
            float area_density=0;
            float visited_areas=0;
            float acc_visited_areas_sum=0;
            float sumNNdist = 0;
            float sumFNdist = 0;

            for (int a = 0; a < Sim1.controller.blackboard.BT_Env.areas_num; a++)
            {
                NNdist = 100000;
                FNdist =0;
                for (int j=0; j< blackboard::Swarm_size;j++)
                {
                    if ((Sim1.SwarmX[i*blackboard::Swarm_size+j]) >= (Sim1.controller.blackboard.BT_Env.areas_X[a]) &&
                        (Sim1.SwarmX[i*blackboard::Swarm_size+j]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_X[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a]) &&
                        (Sim1.SwarmY[i*blackboard::Swarm_size+j]) >= (Sim1.controller.blackboard.BT_Env.areas_Y[a]) &&
                        (Sim1.SwarmY[i*blackboard::Swarm_size+j]) <=
                        (Sim1.controller.blackboard.BT_Env.areas_Y[a] + Sim1.controller.blackboard.BT_Env.areas_Width[a])) {
                        area_density += 1;
                        area=1;
                        acc_visited_areas[a]=1;
                    }
                    dist_temp = sqrt(pow((Sim1.SwarmX[i*blackboard::Swarm_size+j])  - (Sim1.controller.blackboard.BT_Env.areas_X[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2) + pow((Sim1.SwarmY[i*blackboard::Swarm_size+j])- (Sim1.controller.blackboard.BT_Env.areas_Y[a]+(Sim1.controller.blackboard.BT_Env.areas_Width[a]/2)), 2));
                    if (dist_temp < NNdist )
                    {
                        NNdist= dist_temp;
                    }
                    if (dist_temp > FNdist )
                    {
                        FNdist= dist_temp;
                    }


                }
                sumNNdist =  sumNNdist + NNdist ;
                sumFNdist =  sumFNdist + FNdist ;

                if (area)
                {
                    visited_areas+=1;
                    area=0;
                }
            }


            agents_inside_area_ratio[i]=(area_density);
            //agents_inside_area_ratio[i]=(area_density/float(blackboard::Swarm_size));
            //visited_areas_ratio_cur[i]=(visited_areas/float(Sim1.controller.blackboard.BT_Env.areas_num));
            //acc_visited_areas_sum= std::accumulate(acc_visited_areas.begin(), acc_visited_areas.end(), 0.0);
            //visited_areas_ratio_acc[i]=(acc_visited_areas_sum/float(Sim1.controller.blackboard.BT_Env.areas_num));
            //areas_agents_centerMassX[i]=((centerMassX[i]+areas_center_x)/2);
            //areas_agents_centerMassY[i]=((centerMassY[i]+areas_center_y)/2);
            areas_avg_NN[i]=(sumNNdist/float(Sim1.controller.blackboard.BT_Env.areas_num));
            areas_avg_FN[i]=(sumFNdist/float(Sim1.controller.blackboard.BT_Env.areas_num));


        }

        swarm_metrics.push_back(agents_inside_area_ratio);
        //swarm_metrics.push_back(visited_areas_ratio_cur);
        //swarm_metrics.push_back(visited_areas_ratio_acc);
        //swarm_metrics.push_back(areas_agents_centerMassX);
        //swarm_metrics.push_back(areas_agents_centerMassY);
        swarm_metrics.push_back(areas_avg_NN);
        swarm_metrics.push_back(areas_avg_FN);

    }



};

#endif //SWARM_SWARMMETRICS_H
