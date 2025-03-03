//
// Created by kholood alharthi on 10/20/21.
//
#include <iostream>
using namespace std;
#include <sstream>
#include <queue>
#include <vector>
#include <algorithm>
#include "Swarm.h"
#include "SwarmMetrics.h"
#include "SwarmMetrics2.h"
#include "blackboard.h"
#include <tuple>
#include <memory>
#include "rapidcsv.h"
#include <set>
#include <string>

#ifndef SWARM_BTEVOLUTION_H
#define SWARM_BTEVOLUTION_H


class BTevolution {

public:
    int population_size ;
    int generations_number;
    vector<vector<blackboard::Node>> population;
    vector<blackboard::env> population_Env;
    vector<blackboard::env> randmon_population_Env;

    vector<tuple<double,vector<blackboard::Node>,blackboard::env>> population_Fit_Env;
    vector<tuple<double,vector<vector<float>>,vector<vector<float>>>> swarm_trajectories;

    vector<pair<double,vector<vector<float>>>> swarm_metrics;
    vector<pair<double,vector<vector<vector<float>>>>> swarm_metrics_agents;

    vector<tuple<double,vector<blackboard::Node>,vector<float>>> population_multiFitness;

    vector<tuple<int,int,double,int,double>> MA_data;

    vector<blackboard::Node> BestBT;
    vector<vector<float>> original_metrics;
    vector<vector<vector<float>>> originals_metrics;
    vector<vector<vector<float>>> originals_metrics_agent_level;
    vector<vector<float>> BestBT_metrics;
    vector<vector<vector<float>>> BestBT_metrics_agents;
    double BestFit;

    vector<float> multi_fit;
    vector<float> optimal_multi_fit;
    vector<vector<float>> best_multi_fit;
    vector<vector<float>> avg_multi_fit;
    vector<vector<float>> max_multi_fit;
    vector<int> sub_tree_nodes ={};
    vector<int> sub_tree_nodes_index ={};
    int child_num_temp;
	string sub_folder="";


    vector<vector<blackboard::Node>> all_BestBT;
    vector<vector<blackboard::Node>> all_worstBT;
    vector<vector<vector<blackboard::Node>>> all_BT;

    vector<double> Best_Fitness;
    vector<double> Avg_Fitness;
    vector<double> Max_Fitness;

    vector<float> original_x;
    vector<float> original_y;

    SwarmMetrics original;
    SwarmMetrics extracted;
    unique_ptr<SwarmMetrics> original2;

    vector<vector<float>> originals_x;
    vector<vector<float>> originals_y;

    vector<SwarmMetrics> originals;
    int originals_num=0;

    vector<vector<double>> Max;
    vector<vector<double>> Min;
    vector<double> Mean;


    bool object_in_env = true;
    bool areas_in_env = true;
    bool fixed_env = true;
    bool obstacle_in_env = false;

    vector<double> Global_Max;
    vector<double> Global_Min;
    vector<double> Global_Max_agent;
    vector<double> Global_Min_agent;
    vector<float> nodes_tick_percentage;
    vector<float> nodes_success_percentage;
    BehaviorTree Generator;
    blackboard::env original_Env;
    blackboard::env originals_Env[5];
    int start_t, end_t;
    vector<int> BT_areas_in_env;
    vector<int> BT_objects_in_env;
    vector<float> dummy_metric={};
    int full_Behaviour_checks=0;
    vector<int> swarm_sizes;
    int Env_max=250,Env_min=-250;

    vector<vector<double>> Best_swarm_metrics;
    unique_ptr<Swarm> s;
    int agent_id1,agent_id2;
    float mutation_force=0.0, crossover_force=0.0;
    bool original_full_Behaviour= false;
    vector<blackboard::Node> original_tree;
    std::vector<double> feature_weights;
    vector<double> smoothedX;
    vector<double> smoothedY;



    BTevolution (vector<blackboard::Node>  BTree, int population_Size, int generations_Number , vector<int> actions, int originals_number)
    {
        originals_num=originals_number;
        population_size=population_Size;
        generations_number=generations_Number;
        Generator.BT_nodes_set_from_list(actions);
        for (int i=0;i<originals_num;i++)
        {
            //SwarmMetrics original_demo;
            originals.push_back(SwarmMetrics());
            originals[i].Sim1.set_BT(BTree);
            originals_Env[i]= originals[i].Sim1.controller.BT_environment;
            originals[i].Sim1.controller.blackboard.BT_Env = originals_Env[i];
            originals[i].set_sizes();
            originals[i].generate_simulation_data();
        }


    }

    BTevolution (vector<blackboard::Node>  BTree, int population_Size, int generations_Number , vector<int> actions)
    {
        population_size=population_Size;
        generations_number=generations_Number;
        Generator.BT_nodes_set_from_list(actions);
        //original.Sim1.set_BT(BTree);
        original.Sim1.set_BT_draw_env(BTree);
        original_Env= original.Sim1.controller.BT_environment;
        original.Sim1.controller.blackboard.BT_Env = original_Env;
        original.set_sizes();
        original.generate_simulation_data();
    }

    BTevolution (int BTraj_ID,vector<blackboard::Node>  BTree, int population_Size, int generations_Number , vector<int> actions)
    {

        original.set_sizes();
        population_size=population_Size;
        generations_number=generations_Number;
        Generator.BT_nodes_set_from_list(actions);

        rapidcsv::Document doc1(sub_folder+"Original_SwarmX"+to_string(BTraj_ID)+".csv", rapidcsv::LabelParams(-1, -1),rapidcsv::SeparatorParams(','));
        rapidcsv::Document doc2(sub_folder+"Original_SwarmY"+to_string(BTraj_ID)+".csv", rapidcsv::LabelParams(-1, -1),rapidcsv::SeparatorParams(','));
        original.Sim1.clear_distances( );

        for (int s=0; s<blackboard::simulation_timesteps;s++)
        {
            vector<double> X  = doc1.GetRow<double>(s);
            vector<double> Y  = doc2.GetRow<double>(s);

            for(int a=0;a<blackboard::Swarm_size;a++)
            {

                original.Sim1.SwarmX[s*blackboard::Swarm_size+a]=X[a];
                original.Sim1.SwarmY[s*blackboard::Swarm_size+a]=Y[a];
                original.Sim1.agent[a].x=X[a];
                original.Sim1.agent[a].y=Y[a];

            }


            original.Sim1.compute_swarm_distance( );
        }

               vector<vector<double>> objects_X;
               vector<vector<double>> objects_Y;


              rapidcsv::Document doc3(sub_folder+"Original_SwarmEnvObjectX"+to_string(BTraj_ID)+".csv", rapidcsv::LabelParams(-1, -1),rapidcsv::SeparatorParams(','));
              rapidcsv::Document doc4(sub_folder+"Original_SwarmEnvObjectY"+to_string(BTraj_ID)+".csv", rapidcsv::LabelParams(-1, -1),rapidcsv::SeparatorParams(','));


               for (int s=0; s<blackboard::simulation_timesteps;s++)
               {

                   std::vector<std::string> XStrings = doc3.GetRow<std::string>(s);
                   std::vector<std::string> YStrings = doc4.GetRow<std::string>(s);


                   // Convert the strings to doubles, ignoring any trailing empty strings
                   std::vector<double> X;
                   for (const auto& cell : XStrings) {
                       if (!cell.empty()) {
                           double value= std::stod(cell);
                           X.push_back(value);
                       }
                   }


                   std::vector<double> Y;
                   for (const auto& cell : YStrings) {
                       if (!cell.empty()) {
                           double value= std::stod(cell);
                           Y.push_back(value);
                       }
                   }

                   objects_X.push_back(X);
                   objects_Y.push_back(Y);
               }
        int objects_number =15;
        original.Sim1.set_BT(BTree,objects_X,objects_Y,objects_number);
        //original.Sim1.set_BT_draw_all_env(BTree); //full env
        original.Sim1.set_BT_draw_env(BTree);
        original_Env= original.Sim1.controller.BT_environment;
        original.Sim1.controller.blackboard.BT_Env = original_Env;

    }


    // Function to smooth x and y with a threshold and moving average
    void smoothTrajectory(double threshold, int windowSize,const std::vector<double>& x, const std::vector<double>& y) {
        std::vector<double> xWindow, yWindow;

        for (size_t i = 0; i < x.size(); ++i) {
            // Add the new values to the window
            xWindow.push_back(x[i]);
            yWindow.push_back(y[i]);

            // Remove the oldest value if window exceeds the size
            if (xWindow.size() > windowSize) xWindow.erase(xWindow.begin());
            if (yWindow.size() > windowSize) yWindow.erase(yWindow.begin());

            // Calculate the moving average for x and y
            double xSum = 0, ySum = 0;
            for (double val : xWindow) xSum += val;
            for (double val : yWindow) ySum += val;

            double smoothedXVal = xSum / xWindow.size();
            double smoothedYVal = ySum / yWindow.size();

            // Only update smoothed values if the change is greater than the threshold
            if (smoothedX.empty() || std::fabs(smoothedXVal - smoothedX.back()) > threshold) {
                smoothedX.push_back(smoothedXVal);
            } else {
                smoothedX.push_back(smoothedX.back());  // Retain the previous value
            }

            if (smoothedY.empty() || std::fabs(smoothedYVal - smoothedY.back()) > threshold) {
                smoothedY.push_back(smoothedYVal);
            } else {
                smoothedY.push_back(smoothedY.back());  // Retain the previous value
            }
        }
    }

    BTevolution (int population_Size ,int generations_Number, vector<int> actions)
    {
        int objects_number =3;
        double threshold = 0.2;
        int windowSize = 15;
        int swarm_size =5;
        generations_number=generations_Number;
        population_size=population_Size;
        Generator.BT_nodes_set_from_list(actions);
        original.set_sizes();

        rapidcsv::Document doc1(sub_folder+"Dots_Env3X.csv", rapidcsv::LabelParams(-1, -1),rapidcsv::SeparatorParams(','));
        rapidcsv::Document doc2(sub_folder+"Dots_Env3Y.csv", rapidcsv::LabelParams(-1, -1),rapidcsv::SeparatorParams(','));
        for (int s=0; s<blackboard::simulation_timesteps;s++)
        {
            vector<double> X  = doc1.GetRow<double>(s);
            vector<double> Y  = doc2.GetRow<double>(s);

            for(int a=0;a<swarm_size;a++)
            {
                original.Sim1.SwarmX[s*swarm_size+a]=X[a];
                original.Sim1.SwarmY[s*swarm_size+a]=Y[a];

            }

        }

        for(int a=0;a<swarm_size;a++)
        {
            vector<double> agent_X;
            vector<double> agent_Y;
            for (int s=0; s<blackboard::simulation_timesteps;s++)
            {
                agent_X.push_back(original.Sim1.SwarmX[s*swarm_size+a]);
                agent_Y.push_back(original.Sim1.SwarmY[s*swarm_size+a]);
            }
            smoothTrajectory(threshold, windowSize,agent_X,agent_Y);
            for (int s=0; s<blackboard::simulation_timesteps;s++)
            {
                original.Sim1.SwarmX[s*swarm_size+a]=smoothedX[s];
                original.Sim1.SwarmY[s*swarm_size+a]=smoothedY[s];
            }
            smoothedX.clear();
            smoothedY.clear();
        }

        original.Sim1.clear_distances(swarm_size);
        for (int s=0; s<blackboard::simulation_timesteps;s++)
        {
            for(int a=0;a<swarm_size;a++)
            {
                original.Sim1.agent[a].x=original.Sim1.SwarmX[s*swarm_size+a];
                original.Sim1.agent[a].y=original.Sim1.SwarmY[s*swarm_size+a];
            }

            original.Sim1.compute_swarm_distance(swarm_size);
        }

        vector<vector<double>> objects_X;
        vector<vector<double>> objects_Y;

        rapidcsv::Document doc3(sub_folder+"Objects_Env3X.csv", rapidcsv::LabelParams(-1, -1),rapidcsv::SeparatorParams(','));
        rapidcsv::Document doc4(sub_folder+"Objects_Env3Y.csv", rapidcsv::LabelParams(-1, -1),rapidcsv::SeparatorParams(','));
        for (int s=0; s<blackboard::simulation_timesteps;s++)
        {
            vector<double> X  = doc3.GetRow<double>(s);
            vector<double> Y  = doc4.GetRow<double>(s);

            objects_X.push_back(X);
            objects_Y.push_back(Y);
        }
        for(int a=0;a<objects_number;a++)
        {
            vector<double> agent_X;
            vector<double> agent_Y;
            for (int s=0; s<blackboard::simulation_timesteps;s++)
            {
                agent_X.push_back(objects_X[s][a]);
                agent_Y.push_back(objects_Y[s][a]);
            }
            smoothTrajectory(threshold, windowSize,agent_X,agent_Y);
            for (int s=0; s<blackboard::simulation_timesteps;s++)
            {
                objects_X[s][a]=smoothedX[s];
                objects_Y[s][a]=smoothedY[s];
            }
            smoothedX.clear();
            smoothedY.clear();
        }

        vector<blackboard::Node> BT;
        BT.push_back({0,4});
        BT.push_back({19,0});
        BT.push_back({16,0});
        BT.push_back({18,0});
        BT.push_back({8,0});
/*        BT.push_back({0,3});
        BT.push_back({1,2});
        BT.push_back({4,0});
        BT.push_back({23,0});
        BT.push_back({18,0});
        BT.push_back({8,0});*/
        original.Sim1.set_BT(BT,objects_X,objects_Y,objects_number);
        //original.Sim1.set_BT_dread_vid_env(BT);
        original_Env= original.Sim1.controller.BT_environment;
        original.Sim1.controller.blackboard.BT_Env = original_Env;


    }


    void set_optimal_original (const std::vector<std::vector<float>>& Swarm_X, const std::vector<std::vector<float>>& Swarm_Y,
                 const std::vector<std::vector<double>>& objects_DataX,const std::vector<std::vector<double>>& objects_DataY,const std::vector<std::vector<std::vector<double>>>& distances)
    {

        for (int s=0; s<blackboard::simulation_timesteps;s++)
        {
            for(int a=0;a<blackboard::Swarm_size;a++)
            {
                original.Sim1.SwarmX[s*blackboard::Swarm_size+a]=Swarm_X[s][a];
                original.Sim1.SwarmY[s*blackboard::Swarm_size+a]=Swarm_Y[s][a];
            }

        }
        original.Sim1.AllDistances=distances;
        original.Sim1.objects_Data_X=objects_DataX;
        original.Sim1.objects_Data_Y=objects_DataY;
    }

    void save_original_rates (int i)
    {
        std::ofstream myfile2;
        myfile2.open (sub_folder+"BT_rates"+to_string((i))+".csv");

        float tick_rate,success_rate;
        for (int i = 0; i < original.Sim1.controller.blackboard.Nodes_tick_times.size(); i++) {
            if (original.Sim1.controller.BT[i].node != 0 and original.Sim1.controller.BT[i].node != 1) {
                if (original.Sim1.controller.blackboard.Nodes_tick_times[i] == 0) {
                    tick_rate = 0;
                    success_rate=0;
                } else {
                    tick_rate = ((float(original.Sim1.controller.blackboard.Nodes_tick_times[i]) /
                                  float(blackboard::simulation_timesteps)));
                    tick_rate /= float(blackboard::Swarm_size);

                    success_rate = ((
                            float(original.Sim1.controller.blackboard.Nodes_success_times[i]) /
                            float(blackboard::simulation_timesteps)));
                    success_rate /= float(blackboard::Swarm_size);

                }
                myfile2 << original.Sim1.controller.BT[i].node << ",";
                myfile2 << tick_rate << ",";
                myfile2 << success_rate ;
            }
            myfile2 << "\n";
        }
        myfile2.close();
    }

    void set_optimal_original ( )
    {

        original.Sim1.controller.blackboard.originalX=original_Env.Initial_swarm_X;
        original.Sim1.controller.blackboard.originalY=original_Env.Initial_swarm_Y;


        original.generate_simulation_data2();

        original_Env.objects_Data_X=original.Sim1.objects_Data_X;
        original_Env.objects_Data_Y=original.Sim1.objects_Data_Y;


        float tick_rate,success_rate;
        for (int i = 0; i < original.Sim1.controller.blackboard.Nodes_tick_times.size(); i++) {
            if (original.Sim1.controller.BT[i].node != 0 and original.Sim1.controller.BT[i].node != 1) {
                if (original.Sim1.controller.blackboard.Nodes_tick_times[i] == 0) {
                    tick_rate -= 10;
                    success_rate=-10;
                } else {
                    tick_rate = ((float(original.Sim1.controller.blackboard.Nodes_tick_times[i]) /
                                  float(blackboard::simulation_timesteps)));
                    tick_rate /= float(blackboard::Swarm_size);

                    success_rate = ((
                            float(original.Sim1.controller.blackboard.Nodes_success_times[i]) /
                            float(blackboard::simulation_timesteps)));
                    success_rate /= float(blackboard::Swarm_size);

                }
                cout<<endl;
                cout<<"Node is : "<<original.Sim1.controller.BT[i].node<<" tick rate is : "<<tick_rate<<endl;
                cout<<"Node is : "<<original.Sim1.controller.BT[i].node<<" success rate is : "<<success_rate<<endl;


            }




        }



    }

    void set_env (vector<blackboard::Node>  BTree, bool env_fixed)
    {
        fixed_env = env_fixed;
        for (int i=0; i<BTree.size(); i++)
        {
            if(BTree[i].node == 10 || BTree[i].node==11
              ||  BTree[i].node == 16 || BTree[i].node==17)
            {
                areas_in_env=true;
            }

            if(BTree[i].node == 18 || BTree[i].node==19)
            {
                object_in_env=true;
            }

            if(BTree[i].node == 20)
            {
                obstacle_in_env=true;
            }
        }
    }

    void Compute_original_Metrics ( )
    {

        original.center_of_mass();
        original.longest_Path();
        original.betaIndex();
        original.max_Radius();
        original.NN_distance_avg();
        original.avg_displacement();
        original.idle_agents();

        if (areas_in_env )
        {
            original.area_metrics();
        }
        if (object_in_env )
        {
            original.object_metrics3();
        }


        original_metrics = original.swarm_metrics;
        original_x.clear();
        original_y.clear();
        for (int i =0; i<blackboard::Swarm_size;i++)
        {
            original_x.push_back(original.Sim1.SwarmX[i]);
            original_y.push_back(original.Sim1.SwarmY[i]);
        }

        for (int i =0; i<original_metrics.size();i++)
        {
            multi_fit.push_back(0.0);
        }

        cout<<"Original Demonstration Metrics Computed";
        cout<<endl;
    }

    void Compute_Ghost_original_Metrics ( )
    {
      

        //// Env only metrics
        original.object_metrics4();
        original.object_areas_metrics();


        original_metrics = original.swarm_metrics;


        unique_ptr<SwarmMetrics> individual;


        for (int i =0; i<original_metrics.size();i++)
        {
            multi_fit.push_back(0.0);
        }

        cout<<"Original Demonstration Metrics Computed";
        cout<<endl;
    }



    void Compute_original_Metrics_agent_level ( )
    {
        original.agents_level_metrics(areas_in_env,object_in_env);

        originals_metrics.push_back(original.agent_level_disp_X);
        originals_metrics.push_back(original.agent_level_disp_Y);

        original.center_of_mass();
        original.longest_Path();
        original.betaIndex();

        original.avg_displacement();
        original.idle_agents();





        if (areas_in_env )
        {
            original.area_metrics();
            originals_metrics.push_back(original.agent_level_area_dist);
        }
        if (object_in_env )
        {
            original.object_metrics();
            originals_metrics.push_back(original.agent_level_objects_dist);
        }
        if (object_in_env  and areas_in_env )
        {
            original.object_areas_metrics();
        }

        original_metrics = original.swarm_metrics;


        original_x.clear();
        original_y.clear();
        for (int i =0; i<blackboard::Swarm_size;i++)
        {
            original_x.push_back(original.Sim1.SwarmX[i]);
            original_y.push_back(original.Sim1.SwarmY[i]);
        }

        for (int i =0; i<originals_metrics[0].size();i++)
        {
            multi_fit.push_back(0.0);

        }



        cout<<"Original Demonstration Metrics Computed";
        cout<<endl;
    }

    void Compute_original_Metrics_DOTS ( )
    {

        original.center_of_mass();
        original.longest_Path();
        original.betaIndex();
        original.max_Radius();
        original.NN_distance_avg();
        original.avg_displacement();
        original.idle_agents();


        if (areas_in_env )
        {
            original.area_metrics();
        }
        if (object_in_env )
        {
            original.object_metrics3();
        }

        original_metrics = original.swarm_metrics;
        original_x.clear();
        original_y.clear();
        for (int i =0; i<blackboard::Swarm_size;i++)
        {
            original_x.push_back(original.Sim1.SwarmX[i]);
            original_y.push_back(original.Sim1.SwarmY[i]);
        }

        for (int i =0; i<original_metrics.size();i++)
        {
            multi_fit.push_back(0.0);

        }



        cout<<"Original Demonstration Metrics Computed";
        cout<<endl;
    }

  

    void generate_population ()
    {
        int max =250, min=-250;

        for (int i =0; i<population_size ; i++)
        {
            Generator.nActions_RandomBT_generator(3);

            population.push_back(Generator.BT);
            if (fixed_env==false)
            {
                Generator.generate_env();
                population_Env.push_back(Generator.BT_environment);
            }
            else
            {
                population_Env.push_back(original_Env);
            }




            Generator.BT.clear();
            Generator.reset_env();
        }

        vector<vector<float>> metrics ={};
        vector<float> metrics1 ={};
        vector<vector<vector<float>>> metrics2 ={};
        for (int i =0; i<population_size ; i++)
        {
            population_Fit_Env.push_back(make_tuple(0.0,population[i],population_Env[i]));
            swarm_metrics.push_back(make_pair(0.0,metrics));
            swarm_metrics_agents.push_back(make_pair(0.0,metrics2));
            swarm_trajectories.push_back(make_tuple(0.0,metrics,metrics));
            population_multiFitness.push_back(make_tuple(0.0,population[i],metrics1));

        }

        for(int i=0;i<blackboard::simulation_timesteps;i++) {

            dummy_metric.push_back(0.0);
        }
        cout<<"Initial population Created";
        cout<<endl;
    }

  

    void Normalize ()
    {
        int swarm_size;
        int index;

        Max.resize(original.swarm_metrics.size());
        double sum=0;
        unique_ptr<SwarmMetrics> individual;
        for (int i =0; i<population_size;i++)
        {

    
            individual.reset (new SwarmMetrics);
            individual->reset();
         
            individual->Sim1.set_BT(get<1>(population_Fit_Env[i]),get<2>(population_Fit_Env[i]));

            individual->generate_simulation_data();

            individual->center_of_mass();
            individual->longest_Path();
            individual->betaIndex();
            individual->max_Radius();
            individual->NN_distance_avg();
            individual->avg_displacement();
            individual->idle_agents();


            if (areas_in_env)
            {
                    individual->area_metrics();
            }
            if (object_in_env)
            {
                individual->object_metrics3();
            }



             //// Env only metrics
         /*    individual->object_metrics4();
             individual->object_areas_metrics();*/



            for (int s=0; s<individual->swarm_metrics.size(); s++)
            {
                sum=0;
                for (int t = 1; t <  blackboard::simulation_timesteps; t++) {

                    sum += (pow(original.swarm_metrics[s][t]  - individual->swarm_metrics[s][t], 2));
                }


                if (std::isnan(sqrt(sum)) || std::isinf(sqrt(sum)))
                {
                } else
                {
                    Max[s].push_back(sqrt(sum));
                }
            }


            //individual.reset();
        }

        for (int s=0; s<original.swarm_metrics.size(); s++)
        {
            int MaxElementIndex2 = max_element(Max[s].begin(), Max[s].end()) - Max[s].begin();

            Global_Max.push_back(Max[s][MaxElementIndex2]);

            cout<<" s "<<s<<" Global_Max "<<Global_Max[s]<<endl;

        }

        BT_areas_in_env.clear();
        BT_objects_in_env.clear();

    }

    void Normalize_agent_level ()
    {

        Min.resize(originals_metrics.size());
        Max.resize(originals_metrics.size());

        for (int s=0; s<originals_metrics.size(); s++)
        {
            for (int m=0; m<originals_metrics[s].size() ; m++)
            {
                int minElementIndex2 = min_element(originals_metrics[s][m].begin(), originals_metrics[s][m].end()) - originals_metrics[s][m].begin();
                int MaxElementIndex2 = max_element(originals_metrics[s][m].begin(), originals_metrics[s][m].end()) - originals_metrics[s][m].begin();

                Min[s].push_back(originals_metrics[s][m][minElementIndex2]);
                Max[s].push_back(originals_metrics[s][m][MaxElementIndex2]);
            }

        }

        vector<vector<vector<float>>> agents_metrics;
        unique_ptr<SwarmMetrics> individual;
        for (int i =0; i<population_size;i++)
        {


            individual.reset (new SwarmMetrics);
            individual->reset();

            individual->Sim1.set_BT(get<1>(population_Fit_Env[i]),get<2>(population_Fit_Env[i]));

            individual->generate_simulation_data();

            individual->agents_level_metrics(areas_in_env,object_in_env);
            agents_metrics.push_back( individual->agent_level_disp_X);
            agents_metrics.push_back( individual->agent_level_disp_Y);


            if (areas_in_env)
            {
                agents_metrics.push_back( individual->agent_level_area_dist);
            }
            if (object_in_env)
            {
                agents_metrics.push_back( individual->agent_level_objects_dist);
            }



            for (int s=0; s<agents_metrics.size(); s++)
            {
                for (int m=0; m<agents_metrics[s].size() ; m++)
                {
                    int minElementIndex2 = min_element(agents_metrics[s][m].begin(), agents_metrics[s][m].end()) - agents_metrics[s][m].begin();
                    int MaxElementIndex2 = max_element(agents_metrics[s][m].begin(), agents_metrics[s][m].end()) - agents_metrics[s][m].begin();

                    Min[s].push_back(agents_metrics[s][m][minElementIndex2]);
                    Max[s].push_back(agents_metrics[s][m][MaxElementIndex2]);
                }

            }
            individual.reset();
            agents_metrics.clear();


        }


        for (int s=0; s<originals_metrics.size(); s++)
        {
            int minElementIndex2 = min_element(Min[s].begin(), Min[s].end()) - Min[s].begin();
            int MaxElementIndex2 = max_element(Max[s].begin(), Max[s].end()) - Max[s].begin();

            Global_Min_agent.push_back(Min[s][minElementIndex2]);
            Global_Max_agent.push_back(Max[s][MaxElementIndex2]);

            for (int m=0; m<originals_metrics[s].size() ; m++)
            {
                for (int i = 0; i <  blackboard::simulation_timesteps; i++) {
                    originals_metrics[s][m][i] = (originals_metrics[s][m][i] - Global_Min_agent[s]) / (Global_Max_agent[s] - Global_Min_agent[s]);

                }
            }

        }



        BT_areas_in_env.clear();
        BT_objects_in_env.clear();

    }


    double similarity_score (vector<vector<float>> Swarm1 ,vector<vector<float>> Swarm2)
    {
        double distGAvg=0.0;
        float distSum= 0.0;
        float distSum2= 0.0;
        float dist= 0.0;
        double min_distance =10000 , max_distance=0.0;
        double sum =0.0;
        for (int s =0; s< Swarm1.size()  ;s++)
        {
            sum =0.0;
            for  (int i=1; i<  blackboard::simulation_timesteps; i++)
            {
                sum = sum + pow(Swarm1[s][i]-Swarm2[s][i],2);
            }

            if (std::isnan(sqrt(sum)) || std::isinf(sqrt(sum)))
            {
            } else
            {
                dist = sqrtl(sum);
                dist = (dist / Global_Max[s]);
                multi_fit.push_back(dist);
                distSum=distSum+(dist);
            /*    if(s<9)
                {
                    distSum=distSum+(dist);
                }
                else
                {
                    distSum2=distSum2+(dist);
                }*/


             /*   if (dist<min_distance)
                {
                    min_distance=dist;
                }
                if (dist>max_distance)
                {
                    max_distance=dist;
                }*/
            }
        }

        distGAvg=(distSum/Swarm1.size());
        return distGAvg;
    }


    double similarity_score_EucTrajectories (vector<vector<float>> Swarm1 ,vector<vector<float>> Swarm2)
    {
        double distGAvg=0.0;
        double distSum= 0.0;
        double dist= 0.0;
        vector<double> Traj_distances(20, 0.0);
        for (int i =0; i< blackboard::simulation_timesteps  ;i++) {

            for (int s = 0; s < blackboard::Swarm_size ; s++) {
                Traj_distances[s] += sqrtl(pow(Swarm1[s + s][i] - Swarm2[s + s][i], 2)
                        +pow(Swarm1[s + s + 1][i] - Swarm2[s + s + 1][i], 2));
            }
        }

        for (int s = 0; s < 20; s++) {
            distSum+= (Traj_distances[s]/blackboard::simulation_timesteps);
        }

        distGAvg=distSum/blackboard::Swarm_size;
        return distGAvg;
    }

    double distance(const vector<double>& p1, const vector<double>& p2) {
        // Euclidean distance between two points in 1D (can be extended to multiple dimensions)
        double dist = 0.0;
        for (size_t i = 0; i < p1.size(); ++i) {
            double diff = p1[i] - p2[i];
            dist += diff * diff;
        }
        return sqrt(dist);
    }

    double min3(double a, double b, double c) {
        return min(a, min(b, c));
    }

    double dtw(vector<vector<double>> trajectory1, vector<vector<double>> trajectory2) {
        const int n = trajectory1.size();
        const int m = trajectory2.size();

        // Create a 2D table to store intermediate distances
        vector<vector<float>> dtw_table(n + 1, vector<float>(m + 1, 0.0));

        // Initialize the first row and column to infinity
        for (int i = 0; i <= n; ++i)
            dtw_table[i][0] = INFINITY;
        for (int j = 0; j <= m; ++j)
            dtw_table[0][j] = INFINITY;

        // Set the origin (0, 0) to 0
        dtw_table[0][0] = 0.0;

        // Calculate DTW distances for each point in both trajectories
        for (int i = 1; i <= n; ++i) {
            for (int j = 1; j <= m; ++j) {

                double cost = distance(trajectory1[i - 1], trajectory2[j - 1]);
                dtw_table[i][j] = cost + min3(dtw_table[i - 1][j],      // from above
                                              dtw_table[i][j - 1],      // from left
                                              dtw_table[i - 1][j - 1]); // from diagonal
            }
        }

        // The final DTW distance is the value at the bottom-right corner of the table
        return dtw_table[n][m];
    }


    double similarity_score_DTW (vector<vector<float>> Swarm1 ,vector<vector<float>> Swarm2)
    {

        // Create a 2D table to store intermediate distances
        vector<vector<float>> dtw_table(blackboard::simulation_timesteps , vector<float>(blackboard::simulation_timesteps , 0.0));


        // Set the origin (0, 0) to 0
        dtw_table[0][0] = 0.0;

        // Calculate DTW distances for each point in both trajectories
        for (int i = 0; i < blackboard::simulation_timesteps; i++) {
            for (int j = 0; j < blackboard::simulation_timesteps; j++) {
                double cost = 0.0;
                double sum  = 0.0;
                for  (int s=0; s< Swarm1.size() ; s++) {

                    sum = sum + pow(Swarm1[s][i]-Swarm2[s][j],2);
                }
                //cost = sqrtl(sum);
                dtw_table[i][j] = sum;

                if(i==0 && j>0)
                    {
                        dtw_table[i][j] += (dtw_table[i][j - 1]);
                    }
                if(i>0 && j==0)
                    {
                        dtw_table[i][j] += (dtw_table[i - 1][j]);
                    }
                if(i>0 && j>0)
                    {
                        dtw_table[i][j] += min3(dtw_table[i - 1][j],      // from above
                                                dtw_table[i][j - 1],      // from left
                                                dtw_table[i - 1][j - 1]); // from diagonal
                    }

            }
        }

        // The final DTW distance is the value at the bottom-right corner of the table
        return sqrtl(dtw_table[blackboard::simulation_timesteps-1][blackboard::simulation_timesteps-1]);
    }

    double similarity_score_DTW (vector<vector<float>> Swarm1 ,vector<vector<float>> Swarm2,
                                 vector<vector<float>> Swarm_traj1 ,vector<vector<float>> Swarm_traj2)
    {
        double metrics_distance=0.0;
        double traj_distance1=0.0;
        double traj_distance2=0.0;


        // Create a 2D table to store intermediate distances
        vector<vector<float>> dtw_table(blackboard::simulation_timesteps , vector<float>(blackboard::simulation_timesteps , 0.0));
        vector<vector<float>> dtw_table2(blackboard::simulation_timesteps , vector<float>(blackboard::simulation_timesteps , 0.0));
        vector<vector<float>> dtw_table3(blackboard::simulation_timesteps , vector<float>(blackboard::simulation_timesteps , 0.0));


        // Set the origin (0, 0) to 0
        dtw_table[0][0] = 0.0;
        dtw_table2[0][0] = 0.0;
        dtw_table3[0][0] = 0.0;


        // Calculate DTW distances for each point in both trajectories
        for (int i = 0; i < blackboard::simulation_timesteps; i++) {
            for (int j = 0; j < blackboard::simulation_timesteps; j++) {

                double sum  = 0.0;
                for  (int s=0; s< Swarm1.size() ; s++) {

                    sum = sum + pow(Swarm1[s][i]-Swarm2[s][j],2);
                }
                //cost = sqrtl(sum);
                dtw_table[i][j] = sum;

                dtw_table2[i][j] = dtw_table2[i][j] + pow(Swarm_traj1[0][i]-Swarm_traj2[0][j],2);
                dtw_table2[i][j] = dtw_table2[i][j] + pow(Swarm_traj1[1][i]-Swarm_traj2[1][j],2);
                dtw_table3[i][j] = dtw_table3[i][j] + pow(Swarm_traj1[2][i]-Swarm_traj2[2][j],2);
                dtw_table3[i][j] = dtw_table3[i][j] + pow(Swarm_traj1[3][i]-Swarm_traj2[3][j],2);

                if(i==0 && j>0)
                {
                    dtw_table[i][j] += (dtw_table[i][j - 1]);
                    dtw_table2[i][j] += (dtw_table2[i][j - 1]);
                    dtw_table3[i][j] += (dtw_table3[i][j - 1]);
                }
                if(i>0 && j==0)
                {
                    dtw_table[i][j] += (dtw_table[i - 1][j]);
                    dtw_table2[i][j] += (dtw_table2[i - 1][j]);
                    dtw_table3[i][j] += (dtw_table3[i - 1][j]);
                }
                if(i>0 && j>0)
                {
                    dtw_table[i][j] += min3(dtw_table[i - 1][j],      // from above
                                             dtw_table[i][j - 1],      // from left
                                             dtw_table[i - 1][j - 1]); // from diagonal
                    dtw_table2[i][j] += min3(dtw_table2[i - 1][j],      // from above
                                            dtw_table2[i][j - 1],      // from left
                                            dtw_table2[i - 1][j - 1]); // from diagonal
                    dtw_table3[i][j] += min3(dtw_table3[i - 1][j],      // from above
                                             dtw_table3[i][j - 1],      // from left
                                             dtw_table3[i - 1][j - 1]); // from diagonal
                }

            }
        }

        // The final DTW distance is the value at the bottom-right corner of the table
        metrics_distance= sqrtl(dtw_table[blackboard::simulation_timesteps-1][blackboard::simulation_timesteps-1]);
        // The final DTW distance is the value at the bottom-right corner of the table
        traj_distance1= sqrtl(dtw_table2[blackboard::simulation_timesteps-1][blackboard::simulation_timesteps-1]);
        traj_distance1 = (traj_distance1 - 0) / (40300 - 0);

        traj_distance2= sqrtl(dtw_table3[blackboard::simulation_timesteps-1][blackboard::simulation_timesteps-1]);
        traj_distance2 = (traj_distance2 - 0) / (40300 - 0);


        return metrics_distance+(traj_distance1+traj_distance2);

    }

    vector<blackboard::Node> remove_unticked_Nodes (vector<blackboard::Node> tree , int index,std::map< int, int > Nodes_ticks_times)
    {
        int child_index;
        switch(tree[index].node){
            case 2 ... 24:
                sub_tree_nodes.push_back(tree[index].node);
                sub_tree_nodes_index.push_back(index);
                index = index -1;
                return remove_unticked_Nodes(tree,index,Nodes_ticks_times);
            case 0:
                child_num_temp = tree[index].child_num;
                child_index = sub_tree_nodes.size()-child_num_temp;
                for(int i=0; i<child_num_temp;i++) {

                    if (sub_tree_nodes[child_index+i]>1 && Nodes_ticks_times[sub_tree_nodes_index[child_index+i]] == 0) {
                        tree[index].child_num-=1;
                        tree[sub_tree_nodes_index[child_index+i]].node = 100;
                    }

                    if ((sub_tree_nodes[child_index+i]==0 || sub_tree_nodes[child_index+i]==1 ) && tree[sub_tree_nodes_index[child_index+i]].child_num <= 0) {
                        tree[index].child_num-=1;
                        tree[sub_tree_nodes_index[child_index+i]].node = 100;
                    }

                }
                for (int i=0; i<child_num_temp ;i++)
                {
                    sub_tree_nodes_index.pop_back();
                    sub_tree_nodes.pop_back();
                }
                sub_tree_nodes.push_back(tree[index].node);
                sub_tree_nodes_index.push_back(index);
                if (index <=0)
                {
                    for (int i=tree.size()-1; i>=0;i--)
                    {
                        if (tree[i].node==100)
                        {
                            tree.erase(tree.begin()+i);
                        }
                    }
                    sub_tree_nodes.clear();
                    sub_tree_nodes_index.clear();
                    return tree;
                } else
                {
                    index = index -1;
                    return remove_unticked_Nodes(tree,index,Nodes_ticks_times);
                }
            case 1:
                child_num_temp = tree[index].child_num;
                child_index = sub_tree_nodes.size()-child_num_temp;
                for(int i=0; i<child_num_temp ;i++) {
                    if (sub_tree_nodes[child_index+i]>1 && Nodes_ticks_times[sub_tree_nodes_index[child_index+i]] == 0) {


                        tree[index].child_num-=1;
                        tree[sub_tree_nodes_index[child_index+i]].node = 100;
                    }

                    if ((sub_tree_nodes[child_index+i]==0 || sub_tree_nodes[child_index+i]==1 ) && tree[sub_tree_nodes_index[child_index+i]].child_num <= 0) {

                        tree[index].child_num-=1;
                        tree[sub_tree_nodes_index[child_index+i]].node = 100;
                    }
                }
                for (int i=0; i<child_num_temp ;i++)
                {
                    sub_tree_nodes_index.pop_back();
                    sub_tree_nodes.pop_back();
                }
                sub_tree_nodes.push_back(tree[index].node);
                sub_tree_nodes_index.push_back(index);
                if (index <=0)
                {
                    for (int i=tree.size()-1; i>=0;i--)
                    {
                        if (tree[i].node==100)
                        {
                            tree.erase(tree.begin()+i);
                        }
                    }
                    sub_tree_nodes.clear();
                    sub_tree_nodes_index.clear();
                    return tree;
                } else
                {
                    index = index -1;
                    return remove_unticked_Nodes(tree,index,Nodes_ticks_times);
                }
        }
    }



    void Compute_population_fitness (int gen )
    {
        double Fitness_sum= 0.0;
        double Fitness_avg= 0.0;
        double random;
        float tree_cost=0;
        unique_ptr<SwarmMetrics> individual;
        vector<vector<blackboard::Node>> BTs;
        vector<float> avg_multi_fitness;
        float fitness=0;
        vector<tuple<double,vector<blackboard::Node>,blackboard::env>> temp_population;
        temp_population = population_Fit_Env;




        for (int f=0; f<original_metrics.size();f++)
        {
            avg_multi_fitness.push_back(0.0);
        }


        int g=0;

        if (gen>0)
        {
           g=3;
           BTs.push_back(get<1>(population_Fit_Env[0]));
           BTs.push_back(get<1>(population_Fit_Env[1]));
           BTs.push_back(get<1>(population_Fit_Env[2]));
        }


        //SwarmMetrics individual[population_size];
        for (int j =g; j<population_size;j++)
        {
                 BTs.push_back(get<1>(population_Fit_Env[j]));
                 fitness=0;
                 individual.reset (new SwarmMetrics);
                 individual->reset();
                 individual->Sim1.controller.blackboard.originalX=original_x;
                 individual->Sim1.controller.blackboard.originalY=original_y;
                 individual->Sim1.set_BT(get<1>(population_Fit_Env[j]),get<2>(population_Fit_Env[j]));
                 individual->generate_simulation_data2();
                 get<2>(population_Fit_Env[j]).objects_Data_X=individual->Sim1.objects_Data_X;
                 get<2>(population_Fit_Env[j]).objects_Data_Y=individual->Sim1.objects_Data_Y;

                 individual->center_of_mass();
                 individual->longest_Path();
                 individual->betaIndex();
                 individual->max_Radius();
                 individual->NN_distance_avg();
                 individual->avg_displacement();
                 individual->idle_agents();
                 individual->avg_displacement_all();
                 if (areas_in_env)
                 {
                     individual->area_metrics();
                 }
                 if (object_in_env)
                 {
                     individual->object_metrics3();
                 }


                 multi_fit.clear();
                 fitness= similarity_score(original_metrics,individual->swarm_metrics);
                 if(individual->AvgVelX_all == 0 && individual->AvgVelY_all == 0)
                 {
                     fitness+=5;
                 }

               get<0>(population_Fit_Env[j])=fitness;
               get<1>(swarm_metrics[j])=individual->swarm_metrics;
               get<1>(swarm_trajectories[j])=individual->Sim1.controller.blackboard.Final_Trajectories_x;
               get<2>(swarm_trajectories[j])=individual->Sim1.controller.blackboard.Final_Trajectories_y;

               if (get<1>(population_Fit_Env[j]).size()>2)
               {
                  get<1>(population_Fit_Env[j]) = remove_unticked_Nodes(get<1>(population_Fit_Env[j]),get<1>(population_Fit_Env[j]).size()-1,individual->Sim1.controller.blackboard.Nodes_tick_times);
               }
           

              tree_cost = (0.003*get<1>(population_Fit_Env[j]).size());
               get<0>(population_Fit_Env[j])+=tree_cost;


            get<0>(swarm_metrics[j])=get<0>(population_Fit_Env[j]);
            get<0>(swarm_trajectories[j])=get<0>(population_Fit_Env[j]);
            get<0>(population_multiFitness[j])=get<0>(population_Fit_Env[j]);
            get<1>(population_multiFitness[j])=get<1>(population_Fit_Env[j]);
            Fitness_sum = Fitness_sum + get<0>(population_Fit_Env[j]);

            for (int f=0; f<multi_fit.size();f++)
            {
                avg_multi_fitness[f]+=multi_fit[f];
            }

            get<2>(population_multiFitness[j])=multi_fit;
        }



        all_BT.push_back(BTs);
        BTs.clear();
        Fitness_avg = Fitness_sum/population_size;
        for (int f=0; f<multi_fit.size();f++)
        {
            avg_multi_fitness[f]/=population_size;
        }
        Avg_Fitness.push_back(Fitness_avg);
        avg_multi_fit.push_back(avg_multi_fitness);
        BT_areas_in_env.clear();
        BT_objects_in_env.clear();
        cout<<"Population size: "<<population.size();
        cout<<endl;
        cout<<"Population Evaluated ";
        cout<<endl;
        cout<<"Avg Fitness: "<<Fitness_avg;
        cout<<endl;

    }

    void Compute_ghost_population_fitness (int gen )
    {
        double Fitness_sum= 0.0;
        double Fitness_avg= 0.0;
        double random;
        double AvgVelX_all_objects,AvgVelY_all_objects;
        float tree_cost=0;
        //SwarmMetrics individual;

        unique_ptr<SwarmMetrics> individual;
        //unique_ptr<SwarmMetrics2> individual2;
        vector<vector<blackboard::Node>> BTs;
        vector<float> avg_multi_fitness;
        float final_avg_fitness, final_fitness ,fitness=0;
        vector<tuple<double,vector<blackboard::Node>,blackboard::env>> temp_population;
        temp_population = population_Fit_Env;
        int swarm_size,final_swarm_size;
        std::map< int, int > BT_Nodes_tick_times;



        for (int f=0; f<original_metrics.size();f++)
        {
            avg_multi_fitness.push_back(0.0);
        }


        int g=0;

    


        if (gen>0)
        {
            g=3;
            BTs.push_back(get<1>(population_Fit_Env[0]));
            BTs.push_back(get<1>(population_Fit_Env[1]));
            BTs.push_back(get<1>(population_Fit_Env[2]));
        }

        int env_index=0;
        //SwarmMetrics individual[population_size];
        //swarm_size= (rand() % 15) + 10;







        for (int j =g; j<population_size;j++)
        {
            BTs.push_back(get<1>(population_Fit_Env[j]));
            final_fitness=1000;
            BT_Nodes_tick_times.clear();
            fitness=0;
            final_avg_fitness=0;



            for (int t=0;t<1;t++)
            {
                swarm_size= (rand() % 10) + 15;

                individual.reset (new SwarmMetrics);
                individual->reset();
             
                //get<2>(population_Fit_Env[j]).swarm_size = swarm_size;
                individual->Sim1.set_BT(get<1>(population_Fit_Env[j]),get<2>(population_Fit_Env[j]));
                individual->generate_simulation_data(swarm_size);




                //// Env only metrics
                individual->avg_displacement_all2(swarm_size);
                individual->object_metrics4();
                individual->object_areas_metrics();


                multi_fit.clear();
                fitness= similarity_score(original_metrics,individual->swarm_metrics);
                if(individual->AvgVelX_all == 0 && individual->AvgVelY_all == 0)
                {
                    fitness+=5;
                }


                final_avg_fitness+=fitness;
                if (fitness<final_fitness)
                {
                    final_fitness=fitness;
                    final_swarm_size = swarm_size;
                    get<1>(swarm_metrics[j])=individual->swarm_metrics;
                    get<1>(swarm_trajectories[j])=individual->Sim1.controller.blackboard.Final_Trajectories_x;
                    get<2>(swarm_trajectories[j])=individual->Sim1.controller.blackboard.Final_Trajectories_y;
                    BT_Nodes_tick_times=individual->Sim1.controller.blackboard.Nodes_tick_times;
                    get<2>(population_Fit_Env[j]).objects_Data_X=individual->Sim1.objects_Data_X;
                    get<2>(population_Fit_Env[j]).objects_Data_Y=individual->Sim1.objects_Data_Y;
                }
            }



          get<0>(population_Fit_Env[j])=(final_avg_fitness);
          get<2>(population_Fit_Env[j]).swarm_size=final_swarm_size;


            if (get<1>(population_Fit_Env[j]).size()>2)
            {
                get<1>(population_Fit_Env[j]) = remove_unticked_Nodes(get<1>(population_Fit_Env[j]),get<1>(population_Fit_Env[j]).size()-1,BT_Nodes_tick_times);
            }

            tree_cost = (0.003*get<1>(population_Fit_Env[j]).size());
            get<0>(population_Fit_Env[j])+=tree_cost;


            get<0>(swarm_metrics[j])=get<0>(population_Fit_Env[j]);
            get<0>(swarm_trajectories[j])=get<0>(population_Fit_Env[j]);
            get<0>(population_multiFitness[j])=get<0>(population_Fit_Env[j]);
            get<1>(population_multiFitness[j])=get<1>(population_Fit_Env[j]);
            Fitness_sum = Fitness_sum + get<0>(population_Fit_Env[j]);

            for (int f=0; f<multi_fit.size();f++)
            {
                avg_multi_fitness[f]+=multi_fit[f];
            }

            get<2>(population_multiFitness[j])=multi_fit;
        }



        all_BT.push_back(BTs);
        BTs.clear();
        Fitness_avg = Fitness_sum/population_size;
        for (int f=0; f<multi_fit.size();f++)
        {
            avg_multi_fitness[f]/=population_size;
        }
        Avg_Fitness.push_back(Fitness_avg);
        avg_multi_fit.push_back(avg_multi_fitness);
        BT_areas_in_env.clear();
        BT_objects_in_env.clear();
        cout<<"Population size: "<<population.size();
        cout<<endl;
        cout<<"Population Evaluated ";
        cout<<endl;
        cout<<"Avg Fitness: "<<Fitness_avg;
        cout<<endl;

    }


    void Compute_population_fitness_agents_level (int gen )
    {
        double Fitness_sum= 0.0;
        double Fitness_avg= 0.0;
        double random;
        //SwarmMetrics individual;
        unique_ptr<SwarmMetrics> individual;
        vector<vector<vector<float>>> swarm_metrics_agent_level;
        //unique_ptr<SwarmMetrics2> individual2;
        vector<vector<blackboard::Node>> BTs;
        vector<float> avg_multi_fitness;

        vector<tuple<double,vector<blackboard::Node>,blackboard::env>> temp_population;
        temp_population = population_Fit_Env;




        for (int f=0; f<original_metrics.size();f++)
        {
            avg_multi_fitness.push_back(0.0);
        }



        int g=0;

        for (int j =0; j<population_size;j++)
        {



            BTs.push_back(get<1>(population_Fit_Env[j]));

            //individual  = std::make_unique<SwarmMetrics>();
            individual.reset (new SwarmMetrics);

            individual->reset();
            individual->Sim1.controller.blackboard.originalX=original_x;
            individual->Sim1.controller.blackboard.originalY=original_y;
            individual->Sim1.set_BT(get<1>(population_Fit_Env[j]),get<2>(population_Fit_Env[j]));

            individual->generate_simulation_data2();
            get<2>(population_Fit_Env[j]).objects_Data_X=individual->Sim1.objects_Data_X;
            get<2>(population_Fit_Env[j]).objects_Data_Y=individual->Sim1.objects_Data_Y;

            individual->agents_level_metrics(areas_in_env,object_in_env);
            swarm_metrics_agent_level.push_back(individual->agent_level_disp_X);
            swarm_metrics_agent_level.push_back(individual->agent_level_disp_Y);


            individual->center_of_mass();
            individual->longest_Path();
            individual->betaIndex();
            individual->avg_displacement();
            individual->idle_agents();
            individual->avg_displacement_all();




            if (areas_in_env)
            {
                    individual->area_metrics();
                    swarm_metrics_agent_level.push_back(individual->agent_level_area_dist);

            }
            if (object_in_env)
            {
                    individual->object_metrics();
                   swarm_metrics_agent_level.push_back(individual->agent_level_objects_dist);

            }

            if (object_in_env and areas_in_env )
            {

                    individual->object_areas_metrics();

            }

            for (int s=0; s<swarm_metrics_agent_level.size(); s++)
            {

                for (int m=0; m<swarm_metrics_agent_level[s].size() ; m++)
                {
                    for (int i = 1; i <  blackboard::simulation_timesteps; i++) {
                        swarm_metrics_agent_level[s][m][i] = (swarm_metrics_agent_level[s][m][i] - Global_Min_agent[s]) / (Global_Max_agent[s] - Global_Min_agent[s]);

                    }
                }
            }

            for (int s=0; s<individual->swarm_metrics.size(); s++)
            {
                for (int i = 0; i <  blackboard::simulation_timesteps; i++) {
                    individual->swarm_metrics[s][i] = (individual->swarm_metrics[s][i] - Global_Min[s]) / (Global_Max[s] - Global_Min[s]);
                }
            }





            multi_fit.clear();
            get<0>(population_Fit_Env[j])= similarity_score(swarm_metrics_agent_level);
            if(individual->AvgVelX_all == 0 && individual->AvgVelY_all == 0)
            {
                get<0>(population_Fit_Env[j])+=10;
            }

            get<1>(swarm_metrics_agents[j])=swarm_metrics_agent_level;
            get<1>(swarm_metrics[j])=individual->swarm_metrics;
            get<1>(swarm_trajectories[j])=individual->Sim1.controller.blackboard.Final_Trajectories_x;
            get<2>(swarm_trajectories[j])=individual->Sim1.controller.blackboard.Final_Trajectories_y;



            if (get<1>(population_Fit_Env[j]).size()>2)
            {
                get<1>(population_Fit_Env[j]) = remove_unticked_Nodes(get<1>(population_Fit_Env[j]),get<1>(population_Fit_Env[j]).size()-1,individual->Sim1.controller.blackboard.Nodes_tick_times);
            }


            if (get<1>(population_Fit_Env[j]).size()>12)
            {
                float tree_cost = (0.05*get<1>(population_Fit_Env[j]).size());
                get<0>(population_Fit_Env[j])+=tree_cost;
            }





            get<0>(swarm_metrics_agents[j])=get<0>(population_Fit_Env[j]);
            get<0>(swarm_metrics[j])=get<0>(population_Fit_Env[j]);
            get<0>(swarm_trajectories[j])=get<0>(population_Fit_Env[j]);
            get<0>(population_multiFitness[j])=get<0>(population_Fit_Env[j]);
            get<1>(population_multiFitness[j])=get<1>(population_Fit_Env[j]);
            Fitness_sum = Fitness_sum + get<0>(population_Fit_Env[j]);

 

            get<2>(population_multiFitness[j])=multi_fit;

     
            swarm_metrics_agent_level.clear();

        }



        all_BT.push_back(BTs);
        BTs.clear();

        Fitness_avg = Fitness_sum/population_size;
 
        Avg_Fitness.push_back(Fitness_avg);
        avg_multi_fit.push_back(avg_multi_fitness);
        BT_areas_in_env.clear();
        BT_objects_in_env.clear();

        cout<<"Population size: "<<population.size();
        cout<<endl;
        cout<<"Population Evaluated ";
        cout<<endl;
        cout<<"Avg Fitness: "<<Fitness_avg;
        cout<<endl;

    }


    void Tournament_selection ()
    {

        //Elitism

        vector<tuple<double,vector<blackboard::Node>,blackboard::env>> temp_population;
        vector<tuple<double,vector<blackboard::Node>,blackboard::env>> temp_population_2;

        temp_population_2 = population_Fit_Env;


       // sort(population_Fit_Env.begin(), population_Fit_Env.end());

        std::sort(begin(population_Fit_Env), end(population_Fit_Env), [](auto const &t1, auto const &t2) {
            return get<0>(t1) < get<0>(t2); // or use a custom compare function
        });

        std::sort(begin(swarm_metrics), end(swarm_metrics), [](auto const &t1, auto const &t2) {
            return get<0>(t1) < get<0>(t2); // or use a custom compare function
        });

        std::sort(begin(swarm_metrics_agents), end(swarm_metrics_agents), [](auto const &t1, auto const &t2) {
            return get<0>(t1) < get<0>(t2); // or use a custom compare function
        });

        std::sort(begin(swarm_trajectories), end(swarm_trajectories), [](auto const &t1, auto const &t2) {
            return get<0>(t1) < get<0>(t2); // or use a custom compare function
        });

        std::sort(begin(population_multiFitness), end(population_multiFitness), [](auto const &t1, auto const &t2) {
            return get<0>(t1) < get<0>(t2); // or use a custom compare function
        });

        temp_population.push_back(population_Fit_Env[0]);
        temp_population.push_back(population_Fit_Env[1]);
        temp_population.push_back(population_Fit_Env[2]);
        //temp_population.push_back(population_Fit_Env[3]);
        //temp_population.push_back(population_Fit_Env[4]);
        //temp_population.push_back(population_Fit_Env[5]);




      /*  for (int s=0;s<original_metrics.size();s++)
        {
            vector<float> m;
            for (int p=0;p<population_size;p++)
            {
                m.push_back(get<2>(population_multiFitness[p])[s]);
            }
            cout<<"idx "<<min_element(m.begin(), m.end()) - m.begin()<<endl;
            m.clear();

        }*/



        optimal_multi_fit=get<2>(population_multiFitness[0]);
        best_multi_fit.push_back(get<2>(population_multiFitness[0]));
        max_multi_fit.push_back(get<2>(population_multiFitness[population_size-1]));
        BestFit=get<0>(population_Fit_Env[0]);
        Best_Fitness.push_back(get<0>(population_Fit_Env[0]));
        Max_Fitness.push_back(get<0>(population_Fit_Env[population_size-1]));
        BestBT=get<1>(population_Fit_Env[0]);
        BestBT_metrics=get<1>(swarm_metrics[0]);
        BestBT_metrics_agents=get<1>(swarm_metrics_agents[0]);
        all_BestBT.push_back(BestBT);
        all_worstBT.push_back(get<1>(population_Fit_Env[population_size-1]));

        cout<<"Best Fitness: "<<get<0>(population_Fit_Env[0]);
        cout<<endl;
        cout<<"Worst Fitness: "<<get<0>(population_Fit_Env[population_size-1]);
        cout<<endl;



        for (int p =3; p<population_size;p++)
        {
            int best_ID;
            double best_Fitness=10000000000;
            int Individual_ID;



            for (int i =0; i<blackboard::tournament_size;i++)
            {
                Individual_ID = (rand() % population_size);


                if (get<0>(temp_population_2[Individual_ID])<best_Fitness)
                {
                    best_Fitness=get<0>(temp_population_2[Individual_ID]);
                    best_ID=Individual_ID;

                }
            }




            temp_population.push_back(temp_population_2[best_ID]);

        }

        //temp_population.push_back(population_Fit_Env[population_size-3]);
        //temp_population.push_back(population_Fit_Env[population_size-2]);
        //temp_population.push_back(population_Fit_Env[population_size-1]);

        cout<<"selection done"<<endl;
        population_Fit_Env.clear();
        population_Fit_Env=temp_population;

    }

    void crossover ( )
    {
        double random;
        vector<blackboard::Node> BT1;
        vector<blackboard::Node> BT2;
        int random_second_parent;
        int random_first_parent;
        int random_second_parent_id;
        int random_first_parent_id;
        int crossover_point1, crossover_point2, subtree_size1, subtree_size2;


        vector<tuple<double,vector<blackboard::Node>,blackboard::env>> temp_population;
        temp_population=population_Fit_Env;
        std::vector<int> parents{ };

        for (int i=3;i<population_size;i++)
        {
            parents.push_back(i);
        }
        //srand(time(0));

        while (!parents.empty())
        {


            random_first_parent_id = (rand() % parents.size());
            random_first_parent = parents[random_first_parent_id];

            do {
                random_second_parent_id = (rand() % parents.size());

            } while (random_second_parent_id==random_first_parent_id);

            random_second_parent = parents[random_second_parent_id];

            if(random_first_parent_id>random_second_parent_id)
            {
                parents.erase(parents.begin()+random_first_parent_id);
                parents.erase(parents.begin()+random_second_parent_id);
            }
            else
            {
                parents.erase(parents.begin()+random_second_parent_id);
                parents.erase(parents.begin()+random_first_parent_id);
            }






            random = ((double) rand() / (RAND_MAX));

            if(random < blackboard::crossover_rate)
            {



                // random_second_parent = randomInRanges(i+1,i+2,i+3,population_size-1);

                BT1 = get<1>(temp_population[random_first_parent]);
                BT2 = get<1>(temp_population[random_second_parent]);




                do {
                    crossover_point1 = (rand() % BT1.size());
                } while (crossover_point1 < 1);

                do {
                    crossover_point2 = (rand() % BT2.size());
                } while (crossover_point2 < 1);



                if (BT1[crossover_point1].child_num ==0)
                {
                    subtree_size1=1;
                }
                else
                {
                    subtree_size1= Subtree_Size(BT1,crossover_point1);
                }

                if (BT2[crossover_point2].child_num ==0)
                {
                    subtree_size2=1;
                }
                else
                {
                    subtree_size2= Subtree_Size(BT2,crossover_point2);
                }


                vector<blackboard::Node> temp1;
                vector<blackboard::Node> temp2;

                temp1.assign(BT1.begin() + crossover_point1, BT1.begin() + crossover_point1 + subtree_size1);
                temp2.assign(BT2.begin() + crossover_point2, BT2.begin() + crossover_point2 + subtree_size2);


                BT1.erase(BT1.begin() + crossover_point1,  BT1.begin() + crossover_point1 + subtree_size1);
                BT2.erase(BT2.begin() + crossover_point2,  BT2.begin() + crossover_point2 + subtree_size2);


                BT1.insert(BT1.begin() + crossover_point1, temp2.begin(),  temp2.end());
                BT2.insert(BT2.begin() + crossover_point2, temp1.begin(),  temp1.end());



                get<1>(population_Fit_Env[random_first_parent])=BT1;
                get<1>(population_Fit_Env[random_second_parent])=BT2;



            }
        }

        cout<<"corossover done"<<endl;


    }

    void crossover_newBT2 ()
    {

        double random;
        vector<blackboard::Node> BT1;
        vector<blackboard::Node> BT2;
        int crossover_point1, crossover_point2, subtree_size1, subtree_size2;
        vector<tuple<double,vector<blackboard::Node>,blackboard::env>> temp_population;
        temp_population=population_Fit_Env;
        std::vector<int> parents{ };
        int max =250, min=-250;

        for (int i=3;i<population_size;i++)
        {

            random = ((double) rand() / (RAND_MAX));
            if(random < blackboard::crossover_rate2)
            {


                BT1 = get<1>(temp_population[i]);
                Generator.nActions_RandomBT_generator(2);
                BT2 =Generator.BT;
                Generator.BT.clear();

      

                do {
                    crossover_point1 = (rand() % BT1.size());
                } while (crossover_point1 < 1);


                if (BT1[crossover_point1].child_num ==0)
                {
                    subtree_size1=1;
                }
                else
                {
                    subtree_size1= Subtree_Size(BT1,crossover_point1);
                }



                BT1.erase(BT1.begin() + crossover_point1,  BT1.begin() + crossover_point1 + subtree_size1);

                BT1.insert(BT1.begin() + crossover_point1, BT2.begin(),  BT2.end());




                get<1>(population_Fit_Env[i])=BT1;




            }
        }

        cout<<"corossover done"<<endl;


    }

    void crossover_newBT ()
    {

        double random;
        vector<blackboard::Node> BT1;
        vector<blackboard::Node> BT2;
        int crossover_point1, crossover_point2, subtree_size1, subtree_size2;
        vector<tuple<double,vector<blackboard::Node>,blackboard::env>> temp_population;
        temp_population=population_Fit_Env;
        std::vector<int> parents{ };

        for (int i=3;i<population_size;i++)
        {

            random = ((double) rand() / (RAND_MAX));
            if(random < blackboard::crossover_rate2)
            {


                BT1 = get<1>(temp_population[i]);
                Generator.nActions_RandomBT_generator(3);
                BT2 =Generator.BT;
                Generator.BT.clear();


                do {
                    crossover_point1 = (rand() % BT1.size());
                } while (crossover_point1 < 1);

                do {
                    crossover_point2 = (rand() % BT2.size());
                } while (crossover_point2 < 1);


                if (BT1[crossover_point1].child_num ==0)
                {
                    subtree_size1=1;
                }
                else
                {
                    subtree_size1= Subtree_Size(BT1,crossover_point1);
                }

                if (BT2[crossover_point2].child_num ==0)
                {
                    subtree_size2=1;
                }
                else
                {
                    subtree_size2= Subtree_Size(BT2,crossover_point2);
                }

                vector<blackboard::Node> temp1;
                vector<blackboard::Node> temp2;

                temp1.assign(BT1.begin() + crossover_point1, BT1.begin() + crossover_point1 + subtree_size1);
                temp2.assign(BT2.begin() + crossover_point2, BT2.begin() + crossover_point2 + subtree_size2);

                BT1.erase(BT1.begin() + crossover_point1,  BT1.begin() + crossover_point1 + subtree_size1);
                BT2.erase(BT2.begin() + crossover_point2,  BT2.begin() + crossover_point2 + subtree_size2);

                BT1.insert(BT1.begin() + crossover_point1, temp2.begin(),  temp2.end());
                BT2.insert(BT2.begin() + crossover_point2, temp1.begin(),  temp1.end());




                get<1>(population_Fit_Env[i])=BT1;




            }
        }

        cout<<"corossover done"<<endl;


    }

    void newBT ()
    {

        double random;
        vector<blackboard::Node> BT1;


        for (int i=3;i<population_size;i++)
        {
            random = ((double) rand() / (RAND_MAX));
            if(random < blackboard::new_BT_rate2)
            {



                Generator.nActions_RandomBT_generator(2);
                BT1 =Generator.BT;
                Generator.BT.clear();


                get<1>(population_Fit_Env[i])=BT1;




            }
        }

        cout<<"corossover done"<<endl;


    }



    int Subtree_Size (vector<blackboard::Node> Tree, int root)
    {
        int subtree_size =0;
        stack<int> subtree;
        for(int i = Tree.size()-1; i>=0; i--)
        {
            if (Tree[i].child_num==0)
            {
                subtree.push(1);
            }
            else
            {
                for(int j =0; j<Tree[i].child_num;j++)
                {
                    subtree_size+=subtree.top();
                    subtree.pop();
                }
                subtree.push(subtree_size+1);
                subtree_size=0;
            }

            if (i==root)
            {
                subtree_size=subtree.top();
                return subtree_size;
            }
        }

        return subtree_size;
    }



    void mutation2 ()
    {
        double random ;
        int mutation_point;
        int mutation_value;


        for ( int i =3; i<population_size; i++)
        {
            random = ((double) rand() / (RAND_MAX));

            if (random < blackboard::mutation_rate) {



                do {

                    mutation_point = (rand() % get<1>(population_Fit_Env[i]).size());
                } while (get<1>(population_Fit_Env[i])[mutation_point].node <= 1);

                do {
                    int random_index = (rand() % Generator.Leaf_nodes.size());
                    mutation_value = Generator.Leaf_nodes[random_index];
                } while (mutation_value == get<1>(population_Fit_Env[i])[mutation_point].node);


                get<1>(population_Fit_Env[i])[mutation_point].node=mutation_value;




            }
        }
    }




    void mutation ( )
    {
        int max =250, min=-250;
        double random ;
        int mutation_point;
        int mutation_value;


        std::vector<int> parents{ };


        for (int i=3;i<population_size;i++)
        {


            random = ((double) rand() / (RAND_MAX));

            int random_index;

            if (random < blackboard::mutation_rate) {




                mutation_point = (rand() % get<1>(population_Fit_Env[i]).size());
                do {
                    if (get<1>(population_Fit_Env[i])[mutation_point].node == 0)
                    {
                        mutation_value = 1;
                    }
                    else if (get<1>(population_Fit_Env[i])[mutation_point].node == 1)
                    {
                        mutation_value = 0;
                    }

                    else
                    {
                        random_index = (rand() % Generator.Leaf_nodes_0.size());
                        mutation_value = Generator.Leaf_nodes_0[random_index];
                    }


                } while (mutation_value == get<1>(population_Fit_Env[i])[mutation_point].node);

                get<1>(population_Fit_Env[i])[mutation_point].node=mutation_value;



            }
        }
    }

    void mutation3 ()
    {
        double random ;
        int mutation_point;
        int mutation_value;


        std::vector<int> parents{ };

        for (int i=3;i<population_size;i++)
        {


            random = ((double) rand() / (RAND_MAX));

            int random_index;

            if (random < blackboard::mutation_rate) {

                do {
                    mutation_point = (rand() % get<1>(population_Fit_Env[i]).size());
                } while (get<1>(population_Fit_Env[i])[mutation_point].node <= 1);


                do {
                    if (get<1>(population_Fit_Env[i])[mutation_point].node == 10 ||
                        get<1>(population_Fit_Env[i])[mutation_point].node == 11 ||
                        get<1>(population_Fit_Env[i])[mutation_point].node == 12 ||
                        get<1>(population_Fit_Env[i])[mutation_point].node == 13 ||
                        get<1>(population_Fit_Env[i])[mutation_point].node == 14 ||
                        get<1>(population_Fit_Env[i])[mutation_point].node == 15 ||
                        get<1>(population_Fit_Env[i])[mutation_point].node == 16 ||
                        get<1>(population_Fit_Env[i])[mutation_point].node == 17 ||
                        get<1>(population_Fit_Env[i])[mutation_point].node == 22 ||
                        get<1>(population_Fit_Env[i])[mutation_point].node == 23 ||
                        get<1>(population_Fit_Env[i])[mutation_point].node == 24)
                    {
                        random_index = (rand() % Generator.conditions_nodes.size());
                        mutation_value = Generator.conditions_nodes[random_index];
                    }
                    else
                    {
                        random_index = (rand() % Generator.non_conditions_nodes.size());
                        mutation_value = Generator.non_conditions_nodes[random_index];
                    }


                } while (mutation_value == get<1>(population_Fit_Env[i])[mutation_point].node);


                get<1>(population_Fit_Env[i])[mutation_point].node=mutation_value;




            }
        }
    }

    void Evolution ()
    {

    
 
        generate_population();
        Compute_original_Metrics();
        //Compute_Ghost_original_Metrics();
        //Compute_original_Metrics_DOTS();
        //Compute_original_Metrics_agent_level();
        Normalize();
        int gen=0;

        cout<<"----------------------------------------------------"<<endl;
        for (int g=0; g<generations_number; g++)
        {

            cout<<"Generation number: ";
            cout<<g+1;
            cout<<endl;
            //Compute_ghost_population_fitness(g);
            Compute_population_fitness(g);
            Tournament_selection();
            crossover( );
            crossover_newBT2();
            mutation();


            cout<<"Best BT is : "<<endl;
            for (int i=0; i<BestBT.size();i++)
            {
                cout<<"Node is : "<<BestBT[i].node<<"child num is : "<<BestBT[i].child_num<<endl;
            }
            cout<<"swarm size  is : "<<get<2>(population_Fit_Env[0]).swarm_size<<endl;
            cout<<"----------------------------------------------------";
            cout<<endl;
        }
        cout<<"End of Evolution";
        cout<<endl;
        cout<<"The Best BT has a fitness = "<<Best_Fitness[generations_number-1];
        cout<<endl;
    

        cout<<"Best BT is : "<<endl;
        for (int i=0; i<BestBT.size();i++)
        {
            cout<<"Node is : "<<BestBT[i].node<<"child num is : "<<BestBT[i].child_num<<endl;
        }




        cout<<"=================================================================================";


    }

    void Evolution2 ()
    {

        //int time_widow = blackboard::simulation_timesteps/(6);
        //int nested_g=0;
        start_t=0;
        end_t=blackboard::simulation_timesteps;
        double sum_best_last=0;
        double running_mean=0;
        //simulation
        int t_last=20;
        double epsilon=0.05;
        // video

        bool no_improvement;


        generate_population();
        Compute_original_Metrics();
        //Compute_original_Metrics_agent_level();
        //Normalize_agent_level();
        Normalize();
        //read_and_compute_original_Metrics(); //for video extraction
        //Compute_original_Metrics_DOTS();  //for video extraction
        //Compute_original_Metrics();
        //check_env_in_BT();
        //Normalize();
        cout<<"----------------------------------------------------"<<endl;

        for (int g=1;g<=t_last;g++)
        {

            cout<<"Generation number: ";
            cout<<g;
            cout<<endl;
            //check_env_in_BT();
            Compute_population_fitness(g);
            //Compute_population_fitness_agents_level(g);


            Tournament_selection();
            crossover( );
            crossover_newBT();
            //mutation();

            cout<<"Best BT is : "<<endl;
            for (int i=0; i<BestBT.size();i++)
            {
                cout<<"Node is : "<<BestBT[i].node<<"child num is : "<<BestBT[i].child_num<<endl;
            }
            cout<<"----------------------------------------------------";
            cout<<endl;
        }

        for (int g=t_last+1;g<=100;g++)
        {

            cout<<"Generation number: ";
            cout<<g;
            cout<<endl;
            Compute_population_fitness(g);
            //Compute_population_fitness_agents_level(g);
            Tournament_selection();

            sum_best_last=0;
            for (int i = 1; i <= t_last; ++i) {
                sum_best_last += Best_Fitness[Best_Fitness.size() - i];
            }

            running_mean = sum_best_last / float(t_last);
            cout<<" running_mean = "<<running_mean<< " current - mean = "<<abs(BestFit - running_mean)<<endl;
            if (abs(BestFit - running_mean) <= epsilon) {
                generations_number=g;
                std::cout << "Termination criterion fulfilled. Stop optimization at generation " << g << std::endl;
                break; // Terminate the loop
            }

           // no_improvement=true;

      /*      for (int i = 1; i <= t_last; ++i) {
                if (abs(Best_Fitness[Best_Fitness.size() - i] - Best_Fitness[Best_Fitness.size() - i - 1]) >= epsilon)  {
                    no_improvement= false; // There was an improvement in one of the previous k iterations
                }
            }
            if (no_improvement) {
                generations_number=g;
                std::cout << "Termination criterion fulfilled. Stop optimization at generation " << g << std::endl;
                break; // Terminate the loop
            }*/


            crossover( );
            crossover_newBT();
            //mutation();

            cout<<"Best BT is : "<<endl;
            for (int i=0; i<BestBT.size();i++)
            {
                cout<<"Node is : "<<BestBT[i].node<<"child num is : "<<BestBT[i].child_num<<endl;
            }
            cout<<"----------------------------------------------------";
            cout<<endl;
        }

        cout<<"End of Evolution";
        cout<<endl;
        cout<<"The Best BT has a fitness = "<<BestFit;
        cout<<endl;
        //BestBT = get<1>(population_Fit_Env[0]);
        //BestFit= get<0>(population_Fit_Env[0]);

        cout<<"Best BT is : "<<endl;
        for (int i=0; i<BestBT.size();i++)
        {
            cout<<"Node is : "<<BestBT[i].node<<"child num is : "<<BestBT[i].child_num<<endl;
        }




        cout<<"=================================================================================";


    }

    void Evolution3 ()
    {

        //int time_widow = blackboard::simulation_timesteps/(6);
        //int nested_g=0;
        start_t=0;
        end_t=blackboard::simulation_timesteps;
        double sum_best_last=0;
        double running_mean=0;
        //simulation
        int t_last=20;
        double epsilon=0.05;
        // video

        bool no_improvement;


        generate_population();
        //read_and_compute_original_Metrics(); //for video extraction
        Compute_original_Metrics_DOTS();  //for video extraction
        //Compute_original_Metrics();
        Normalize();
        cout<<"----------------------------------------------------"<<endl;


        for (int g=1;g<=5;g++)
        {

            cout<<"Generation number: ";
            cout<<g;
            cout<<endl;
            Compute_population_fitness(g);
            Tournament_selection();

            if (BestFit <=1) {
                generations_number=g;
                std::cout << "Termination criterion fulfilled. Stop optimization at generation " << g << std::endl;
                break; // Terminate the loop
            }

            crossover( );
            crossover_newBT();
            //mutation();

            cout<<"Best BT is : "<<endl;
            for (int i=0; i<BestBT.size();i++)
            {
                cout<<"Node is : "<<BestBT[i].node<<"child num is : "<<BestBT[i].child_num<<endl;
            }
            cout<<"----------------------------------------------------";
            cout<<endl;
        }

        cout<<"End of Evolution";
        cout<<endl;
        cout<<"The Best BT has a fitness = "<<BestFit;
        cout<<endl;
        //BestBT = get<1>(population_Fit_Env[0]);
        //BestFit= get<0>(population_Fit_Env[0]);

        cout<<"Best BT is : "<<endl;
        for (int i=0; i<BestBT.size();i++)
        {
            cout<<"Node is : "<<BestBT[i].node<<"child num is : "<<BestBT[i].child_num<<endl;
        }




        cout<<"=================================================================================";


    }

    void save_final_sim_data (int i,vector<blackboard::Node>  BTree)
    {
        int original_swarm_size=20;
        // Save original BT data
        std::ofstream myfile;
        myfile.open (sub_folder+Original_SwarmX"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<original_swarm_size; i++)
            {            myfile << original.Sim1.SwarmX[j*original_swarm_size+i];
                if (i!=original_swarm_size-1)
                {
                    myfile<< ",";
                }
            }
            myfile << "\n";
        }
        myfile.close();

        std::ofstream myfile2;
        myfile2.open (sub_folder+Original_SwarmY"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<original_swarm_size; i++)
            {
                myfile2 << original.Sim1.SwarmY[j*original_swarm_size+i] ;
                if (i!=original_swarm_size-1)
                {
                    myfile2<< ",";
                }
            }
            myfile2 << "\n";
        }
        myfile2.close();




        std::ofstream myfile3;
        myfile3.open (sub_folder+Original_SwarmEnv"+to_string((i))+".csv");
        if(original.Sim1.controller.blackboard.BT_Env.areas_in_env)
        {
            myfile3 << 1 << "," ;
            std::ofstream myfile4;
            myfile4.open (sub_folder+Original_SwarmEnvAreasX"+to_string((i))+".csv");
            std::ofstream myfile5;
            myfile5.open (sub_folder+Original_SwarmEnvAreasY"+to_string((i))+".csv");
            std::ofstream myfile6;
            myfile6.open (sub_folder+Original_SwarmEnvAreasWidth"+to_string((i))+".csv");
            for(int i=0; i<original.Sim1.controller.blackboard.BT_Env.areas_num; i++)
            {
                myfile4 << original.Sim1.controller.blackboard.BT_Env.areas_X[i]<< "," ;
                myfile5 << original.Sim1.controller.blackboard.BT_Env.areas_Y[i]<< "," ;
                myfile6 << original.Sim1.controller.blackboard.BT_Env.areas_Width[i]<< "," ;
            }
            myfile4.close();
            myfile5.close();
            myfile6.close();

        }
        else
            myfile3 << 0 << "," ;
        if(original.Sim1.controller.blackboard.BT_Env.objects_in_env)
        {
            myfile3 << 1 << "," ;

            std::ofstream myfile4;
            myfile4.open (sub_folder+Original_SwarmEnvObjectX"+to_string((i))+".csv");
            std::ofstream myfile5;
            myfile5.open (sub_folder+Original_SwarmEnvObjectY"+to_string((i))+".csv");
            for ( int i=0; i<blackboard::simulation_timesteps; i++){
                for(int j=0; j<original.Sim1.controller.blackboard.BT_Env.objects_num; j++)
                {
                    myfile4 << original.Sim1.objects_Data_X[i][j]<< "," ;
                    myfile5 << original.Sim1.objects_Data_Y[i][j]<< "," ;

                }
                myfile4 << "\n";
                myfile5 << "\n";

            }
            myfile4.close();
            myfile5.close();


        }
        else
            myfile3 << 0 << "," ;
        if(original.Sim1.controller.blackboard.BT_Env.obstacles_in_env)
        {
            myfile3 << 1 << "," ;
            std::ofstream myfile4;
            myfile4.open (sub_folder+Original_SwarmEnvObstaclesX"+to_string((i))+".csv");
            std::ofstream myfile5;
            myfile5.open (sub_folder+Original_SwarmEnvObstaclesY"+to_string((i))+".csv");
            std::ofstream myfile6;
            myfile6.open (sub_folder+Original_SwarmEnvObstaclesX2"+to_string((i))+".csv");
            std::ofstream myfile7;
            myfile7.open (sub_folder+Original_SwarmEnvObstaclesY2"+to_string((i))+".csv");
            for(int i=0; i<original.Sim1.controller.blackboard.BT_Env.obstacles_num; i++)
            {
                myfile4 << original.Sim1.controller.blackboard.BT_Env.obstacles_X[i]<< "," ;
                myfile5 << original.Sim1.controller.blackboard.BT_Env.obstacles_Y[i]<< "," ;
                myfile6 << original.Sim1.controller.blackboard.BT_Env.obstacles_X2[i]<< "," ;
                myfile7 << original.Sim1.controller.blackboard.BT_Env.obstacles_Y2[i]<< "," ;

            }
            myfile4.close();
            myfile5.close();
            myfile6.close();
            myfile7.close();


        }
        else
            myfile3 << 0 << "," ;
        myfile3.close();

        std::ofstream myfile8;
        myfile8.open (sub_folder+Original_BT"+to_string((i))+".csv");
        for ( int j=0; j<original.Sim1.controller.BT.size(); j++) {
            myfile8 << original.Sim1.controller.BT[j].node << ",";
        }
        myfile8 << "\n";
        for ( int j=0; j<original.Sim1.controller.BT.size(); j++) {
            myfile8 << original.Sim1.controller.BT[j].child_num << ",";
        }
        myfile8.close();

        save_original_rates (i);


        // Save imitated BT data
        std::ofstream myfile9;
        myfile9.open (sub_folder+imitated_SwarmX"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<blackboard::Swarm_size; i++)
            {            myfile9 << get<1>(swarm_trajectories[0])[j][i];
                if (i!=blackboard::Swarm_size-1)
                {
                    myfile9<< ",";
                }
            }
            myfile9 << "\n";
        }
        myfile9.close();


        std::ofstream myfile12;
        myfile12.open (sub_folder+imitated_SwarmY"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<blackboard::Swarm_size; i++)
            {
                myfile12 << get<2>(swarm_trajectories[0])[j][i];
                if (i!=blackboard::Swarm_size-1)
                {
                    myfile12<< ",";
                }
            }
            myfile12 << "\n";
        }
        myfile12.close();



        std::ofstream myfile13;
        myfile13.open (sub_folder+imitated_SwarmEnv"+to_string((i))+".csv");
        if(get<2>(population_Fit_Env[0]).areas_in_env)
        {
            myfile13 << 1 << "," ;
            std::ofstream myfile14;
            myfile14.open (sub_folder+imitated_SwarmEnvAreasX"+to_string((i))+".csv");
            std::ofstream myfile15;
            myfile15.open (sub_folder+imitated_SwarmEnvAreasY"+to_string((i))+".csv");
            std::ofstream myfile16;
            myfile16.open (sub_folder+imitated_SwarmEnvAreasWidth"+to_string((i))+".csv");
            for(int i=0; i<get<2>(population_Fit_Env[0]).areas_num; i++)
            {
                myfile14 <<get<2>(population_Fit_Env[0]).areas_X[i]<< "," ;
                myfile15 << get<2>(population_Fit_Env[0]).areas_Y[i]<< "," ;
                myfile16 << get<2>(population_Fit_Env[0]).areas_Width[i]<< "," ;
            }
            myfile14.close();
            myfile15.close();
            myfile16.close();
        }
        else
            myfile13 << 0 << "," ;
        if(get<2>(population_Fit_Env[0]).objects_in_env)
        {
            myfile13 << 1 << "," ;

            std::ofstream myfile14;
            myfile14.open (sub_folder+imitated_SwarmEnvObjectX"+to_string((i))+".csv");
            std::ofstream myfile15;
            myfile15.open (sub_folder+imitated_SwarmEnvObjectY"+to_string((i))+".csv");
            for ( int i=0; i<blackboard::simulation_timesteps; i++){
                for(int j=0; j<get<2>(population_Fit_Env[0]).objects_num; j++)
                {
                    myfile14 << get<2>(population_Fit_Env[0]).objects_Data_X[i][j]<< "," ;
                    myfile15 << get<2>(population_Fit_Env[0]).objects_Data_Y[i][j]<< "," ;

                }
                myfile14 << "\n";
                myfile15 << "\n";

            }
            myfile14.close();
            myfile15.close();

        }
        else
            myfile13 << 0 << "," ;
        if(get<2>(population_Fit_Env[0]).obstacles_in_env)
        {
            myfile13 << 1 << "," ;
            std::ofstream myfile14;
            myfile14.open (sub_folder+imitated_SwarmEnvObstaclesX"+to_string((i))+".csv");
            std::ofstream myfile15;
            myfile15.open (sub_folder+imitated_SwarmEnvObstaclesY"+to_string((i))+".csv");
            std::ofstream myfile16;
            myfile16.open (sub_folder+imitated_SwarmEnvObstaclesX2"+to_string((i))+".csv");
            std::ofstream myfile17;
            myfile17.open (sub_folder+imitated_SwarmEnvObstaclesY2"+to_string((i))+".csv");
            for(int i=0; i<get<2>(population_Fit_Env[0]).obstacles_num; i++)
            {
                myfile14 << get<2>(population_Fit_Env[0]).obstacles_X[i]<< "," ;
                myfile15 << get<2>(population_Fit_Env[0]).obstacles_Y[i]<< "," ;
                myfile16 << get<2>(population_Fit_Env[0]).obstacles_X2[i]<< "," ;
                myfile17 << get<2>(population_Fit_Env[0]).obstacles_Y2[i]<< "," ;

            }
            myfile14.close();
            myfile15.close();
            myfile16.close();
            myfile17.close();

        }
        else
            myfile13 << 0 << "," ;
        myfile13.close();
        std::ofstream myfile18;
        myfile18.open (sub_folder+imitated_BT"+to_string((i))+".csv");
        for ( int j=0; j<BTree.size(); j++) {
            myfile18 << BTree[j].node << ",";

        }
        myfile18 << "\n";
        for ( int j=0; j<BTree.size(); j++) {
            myfile18 << BTree[j].child_num << ",";
        }
        myfile18.close();

        std::ofstream myfile118;
        myfile118.open (sub_folder+Best_BTs_"+to_string((i))+".csv");
        for (int b=0;b<generations_number;b++)
        {
            for ( int j=0; j<all_BestBT[b].size(); j++) {
                myfile118 << all_BestBT[b][j].node;
                if (j!=all_BestBT[b].size()-1)
                {
                    myfile118<< ",";
                }

            }
            myfile118 << "\n";
            for ( int j=0; j<all_BestBT[b].size(); j++) {
                myfile118 << all_BestBT[b][j].child_num;
                if (j!=all_BestBT[b].size()-1)
                {
                    myfile118<< ",";
                }
            }
            myfile118 << "\n";
        }
        myfile118.close();

        std::ofstream myfile30;
        myfile30.open (sub_folder+Worst_BTs_"+to_string((i))+".csv");
        for (int b=0;b<generations_number;b++)
        {
            for ( int j=0; j<all_worstBT[b].size(); j++) {
                myfile30 << all_worstBT[b][j].node;
                if (j!=all_worstBT[b].size()-1)
                {
                    myfile30<< ",";
                }

            }
            myfile30 << "\n";
            for ( int j=0; j<all_worstBT[b].size(); j++) {
                myfile30 << all_worstBT[b][j].child_num;
                if (j!=all_worstBT[b].size()-1)
                {
                    myfile30<< ",";
                }
            }
            myfile30 << "\n";
        }
        myfile30.close();

        std::ofstream myfile19;
        myfile19.open (sub_folder+BestFitness"+to_string((i))+".csv");

        for(int i=0; i<Best_Fitness.size(); i++)
        {            myfile19 << Best_Fitness[i] << ",";
        }
        myfile19 << "\n";
        myfile19.close();

        std::ofstream myfile20;
        myfile20.open (sub_folder+Avg_Fitness"+to_string((i))+".csv");

        for(int i=0; i<Avg_Fitness.size(); i++)
        {            myfile20 << Avg_Fitness[i] << ",";
        }
        myfile20 << "\n";
        myfile20.close();

        std::ofstream myfile40;
        myfile40.open (sub_folder+Max_Fitness"+to_string((i))+".csv");

        for(int i=0; i<Max_Fitness.size(); i++)
        {            myfile40 << Max_Fitness[i] << ",";
        }
        myfile40 << "\n";
        myfile40.close();

        std::ofstream myfile21;
        myfile21.open (sub_folder+BestBT_Metrics"+std::to_string(i)+".csv");
        for (int i=0; i< blackboard::simulation_timesteps ;i++)
        {
            for ( int j=0; j<BestBT_metrics.size(); j++){
                myfile21 << BestBT_metrics[j][i]<< ","  ;
            }
            myfile21 <<"\n"  ;
        }
        myfile21.close();

        std::ofstream myfile22;
        myfile22.open (sub_folder+OriginalBT_Metrics"+std::to_string(i)+".csv");

        for (int i=0; i< blackboard::simulation_timesteps ;i++)
        {
            for ( int j=0; j<original_metrics.size(); j++){
                myfile22 << original_metrics[j][i]<< ",";
            }
            myfile22 << "\n"  ;
        }
        myfile22.close();

        std::ofstream myfile23;
        myfile23.open (sub_folder+All_BTs"+std::to_string(i)+".csv");

        for (int i=0; i< all_BT.size() ;i++)
        {
            for ( int j=0; j<all_BT[i].size(); j++){

                for ( int k=0; k<all_BT[i][j].size(); k++){

                    myfile23 << all_BT[i][j][k].node<< ",";
                }
                myfile23 << "\n";

                for ( int k=0; k<all_BT[i][j].size(); k++){

                    myfile23 << all_BT[i][j][k].child_num<< ",";
                }
                myfile23 << "\n";
            }
        }
        myfile23.close();

        std::ofstream myfile24;
        myfile24.open(sub_folder+Best_Metrics_Fitness" +
                      std::to_string(i) + ".csv");

        for (int i = 0; i < best_multi_fit.size(); i++) {
            for (int j = 0; j < best_multi_fit[i].size(); j++) {

                myfile24 << best_multi_fit[i][j] << ",";
            }
            myfile24 << "\n";
        }
        myfile24.close();

       std::ofstream myfile25;
        myfile25.open(sub_folder+Avg_Metrics_Fitness_" +
                      std::to_string(i) + ".csv");

        for (int i = 0; i < avg_multi_fit.size(); i++) {
            for (int j = 0; j < avg_multi_fit[i].size(); j++) {

                myfile25 << avg_multi_fit[i][j] << ",";
            }
            myfile25 << "\n";
        }

        myfile25.close();

        std::ofstream myfile26;
        myfile26.open(sub_folder+Worst_Metrics_Fitness_" +
                      std::to_string(i) + ".csv");

        for (int i = 0; i < max_multi_fit.size(); i++) {
            for (int j = 0; j < max_multi_fit[i].size(); j++) {

                myfile26 << max_multi_fit[i][j] << ",";
            }
            myfile26 << "\n";
        }

        myfile26.close();

        if (MA_data.size()>0)
        {
            std::ofstream myfile27;
            myfile27.open(sub_folder+local_search_data_" +
                          std::to_string(i) + ".csv");

            for (int d = 0; d < MA_data.size(); d++) {
                myfile27 <<get<0>(MA_data[d]) << ",";
                myfile27 <<get<1>(MA_data[d]) << ",";
                myfile27 <<get<2>(MA_data[d]) << ",";
                myfile27 <<get<3>(MA_data[d]) << ",";
                myfile27 <<get<4>(MA_data[d]);
                myfile27 << "\n";
            }

            myfile27.close();
        }
    }

    void save_final_sim_data_ghost_swarm (int i,vector<blackboard::Node>  BTree)
    {
        int original_swarm_size=20;
        // Save original BT data
        std::ofstream myfile;
        myfile.open (sub_folder+Original_SwarmX"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<original_swarm_size; i++)
            {            myfile << original.Sim1.SwarmX[j*original_swarm_size+i];
                if (i!=original_swarm_size-1)
                {
                    myfile<< ",";
                }
            }
            myfile << "\n";
        }
        myfile.close();

        std::ofstream myfile2;
        myfile2.open (sub_folder+Original_SwarmY"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<original_swarm_size; i++)
            {
                myfile2 << original.Sim1.SwarmY[j*original_swarm_size+i] ;
                if (i!=original_swarm_size-1)
                {
                    myfile2<< ",";
                }
            }
            myfile2 << "\n";
        }
        myfile2.close();




        std::ofstream myfile3;
        myfile3.open (sub_folder+Original_SwarmEnv"+to_string((i))+".csv");
        if(original.Sim1.controller.blackboard.BT_Env.areas_in_env)
        {
            myfile3 << 1 << "," ;
            std::ofstream myfile4;
            myfile4.open (sub_folder+Original_SwarmEnvAreasX"+to_string((i))+".csv");
            std::ofstream myfile5;
            myfile5.open (sub_folder+Original_SwarmEnvAreasY"+to_string((i))+".csv");
            std::ofstream myfile6;
            myfile6.open (sub_folder+Original_SwarmEnvAreasWidth"+to_string((i))+".csv");
            for(int i=0; i<original.Sim1.controller.blackboard.BT_Env.areas_num; i++)
            {
                myfile4 << original.Sim1.controller.blackboard.BT_Env.areas_X[i]<< "," ;
                myfile5 << original.Sim1.controller.blackboard.BT_Env.areas_Y[i]<< "," ;
                myfile6 << original.Sim1.controller.blackboard.BT_Env.areas_Width[i]<< "," ;
            }
            myfile4.close();
            myfile5.close();
            myfile6.close();

        }
        else
            myfile3 << 0 << "," ;
        if(original.Sim1.controller.blackboard.BT_Env.objects_in_env)
        {
            myfile3 << 1 << "," ;

            std::ofstream myfile4;
            myfile4.open (sub_folder+Original_SwarmEnvObjectX"+to_string((i))+".csv");
            std::ofstream myfile5;
            myfile5.open (sub_folder+Original_SwarmEnvObjectY"+to_string((i))+".csv");
            for ( int i=0; i<blackboard::simulation_timesteps; i++){
                for(int j=0; j<original.Sim1.controller.blackboard.BT_Env.objects_num; j++)
                {
                    myfile4 << original.Sim1.objects_Data_X[i][j]<< "," ;
                    myfile5 << original.Sim1.objects_Data_Y[i][j]<< "," ;

                }
                myfile4 << "\n";
                myfile5 << "\n";

            }
            myfile4.close();
            myfile5.close();


        }
        else
            myfile3 << 0 << "," ;
        if(original.Sim1.controller.blackboard.BT_Env.obstacles_in_env)
        {
            myfile3 << 1 << "," ;
            std::ofstream myfile4;
            myfile4.open (sub_folder+Original_SwarmEnvObstaclesX"+to_string((i))+".csv");
            std::ofstream myfile5;
            myfile5.open (sub_folder+Original_SwarmEnvObstaclesY"+to_string((i))+".csv");
            std::ofstream myfile6;
            myfile6.open (sub_folder+Original_SwarmEnvObstaclesX2"+to_string((i))+".csv");
            std::ofstream myfile7;
            myfile7.open (sub_folder+Original_SwarmEnvObstaclesY2"+to_string((i))+".csv");
            for(int i=0; i<original.Sim1.controller.blackboard.BT_Env.obstacles_num; i++)
            {
                myfile4 << original.Sim1.controller.blackboard.BT_Env.obstacles_X[i]<< "," ;
                myfile5 << original.Sim1.controller.blackboard.BT_Env.obstacles_Y[i]<< "," ;
                myfile6 << original.Sim1.controller.blackboard.BT_Env.obstacles_X2[i]<< "," ;
                myfile7 << original.Sim1.controller.blackboard.BT_Env.obstacles_Y2[i]<< "," ;

            }
            myfile4.close();
            myfile5.close();
            myfile6.close();
            myfile7.close();


        }
        else
            myfile3 << 0 << "," ;
        myfile3.close();

        std::ofstream myfile8;
        myfile8.open (sub_folder+Original_BT"+to_string((i))+".csv");
        for ( int j=0; j<original.Sim1.controller.BT.size(); j++) {
            myfile8 << original.Sim1.controller.BT[j].node << ",";
        }
        myfile8 << "\n";
        for ( int j=0; j<original.Sim1.controller.BT.size(); j++) {
            myfile8 << original.Sim1.controller.BT[j].child_num << ",";
        }
        myfile8.close();

        //save_original_rates (i);


        // Save imitated BT data
        int imitated_swarm_size=get<2>(population_Fit_Env[0]).swarm_size;
        std::ofstream myfile9;
        myfile9.open (sub_folder+imitated_SwarmX"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<imitated_swarm_size; i++)
            {            myfile9 << get<1>(swarm_trajectories[0])[j][i];
                if (i!=imitated_swarm_size-1)
                {
                    myfile9<< ",";
                }
            }
            myfile9 << "\n";
        }
        myfile9.close();


        std::ofstream myfile12;
        myfile12.open (sub_folder+imitated_SwarmY"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<imitated_swarm_size; i++)
            {
                myfile12 << get<2>(swarm_trajectories[0])[j][i];
                if (i!=imitated_swarm_size-1)
                {
                    myfile12<< ",";
                }
            }
            myfile12 << "\n";
        }
        myfile12.close();



        std::ofstream myfile13;
        myfile13.open (sub_folder+imitated_SwarmEnv"+to_string((i))+".csv");
        if(get<2>(population_Fit_Env[0]).areas_in_env)
        {
            myfile13 << 1 << "," ;
            std::ofstream myfile14;
            myfile14.open (sub_folder+imitated_SwarmEnvAreasX"+to_string((i))+".csv");
            std::ofstream myfile15;
            myfile15.open (sub_folder+imitated_SwarmEnvAreasY"+to_string((i))+".csv");
            std::ofstream myfile16;
            myfile16.open (sub_folder+imitated_SwarmEnvAreasWidth"+to_string((i))+".csv");
            for(int i=0; i<get<2>(population_Fit_Env[0]).areas_num; i++)
            {
                myfile14 <<get<2>(population_Fit_Env[0]).areas_X[i]<< "," ;
                myfile15 << get<2>(population_Fit_Env[0]).areas_Y[i]<< "," ;
                myfile16 << get<2>(population_Fit_Env[0]).areas_Width[i]<< "," ;
            }
            myfile14.close();
            myfile15.close();
            myfile16.close();
        }
        else
            myfile13 << 0 << "," ;
        if(get<2>(population_Fit_Env[0]).objects_in_env)
        {
            myfile13 << 1 << "," ;

            std::ofstream myfile14;
            myfile14.open (sub_folder+imitated_SwarmEnvObjectX"+to_string((i))+".csv");
            std::ofstream myfile15;
            myfile15.open (sub_folder+imitated_SwarmEnvObjectY"+to_string((i))+".csv");
            for ( int i=0; i<blackboard::simulation_timesteps; i++){
                for(int j=0; j<get<2>(population_Fit_Env[0]).objects_num; j++)
                {
                    myfile14 << get<2>(population_Fit_Env[0]).objects_Data_X[i][j]<< "," ;
                    myfile15 << get<2>(population_Fit_Env[0]).objects_Data_Y[i][j]<< "," ;

                }
                myfile14 << "\n";
                myfile15 << "\n";

            }
            myfile14.close();
            myfile15.close();

        }
        else
            myfile13 << 0 << "," ;
        if(get<2>(population_Fit_Env[0]).obstacles_in_env)
        {
            myfile13 << 1 << "," ;
            std::ofstream myfile14;
            myfile14.open (sub_folder+imitated_SwarmEnvObstaclesX"+to_string((i))+".csv");
            std::ofstream myfile15;
            myfile15.open (sub_folder+imitated_SwarmEnvObstaclesY"+to_string((i))+".csv");
            std::ofstream myfile16;
            myfile16.open (sub_folder+imitated_SwarmEnvObstaclesX2"+to_string((i))+".csv");
            std::ofstream myfile17;
            myfile17.open (sub_folder+imitated_SwarmEnvObstaclesY2"+to_string((i))+".csv");
            for(int i=0; i<get<2>(population_Fit_Env[0]).obstacles_num; i++)
            {
                myfile14 << get<2>(population_Fit_Env[0]).obstacles_X[i]<< "," ;
                myfile15 << get<2>(population_Fit_Env[0]).obstacles_Y[i]<< "," ;
                myfile16 << get<2>(population_Fit_Env[0]).obstacles_X2[i]<< "," ;
                myfile17 << get<2>(population_Fit_Env[0]).obstacles_Y2[i]<< "," ;

            }
            myfile14.close();
            myfile15.close();
            myfile16.close();
            myfile17.close();

        }
        else
            myfile13 << 0 << "," ;
        myfile13.close();
        std::ofstream myfile18;
        myfile18.open (sub_folder+imitated_BT"+to_string((i))+".csv");
        for ( int j=0; j<BTree.size(); j++) {
            myfile18 << BTree[j].node << ",";

        }
        myfile18 << "\n";
        for ( int j=0; j<BTree.size(); j++) {
            myfile18 << BTree[j].child_num << ",";
        }
        myfile18.close();

        std::ofstream myfile118;
        myfile118.open (sub_folder+Best_BTs_"+to_string((i))+".csv");
        for (int b=0;b<generations_number;b++)
        {
            for ( int j=0; j<all_BestBT[b].size(); j++) {
                myfile118 << all_BestBT[b][j].node;
                if (j!=all_BestBT[b].size()-1)
                {
                    myfile118<< ",";
                }

            }
            myfile118 << "\n";
            for ( int j=0; j<all_BestBT[b].size(); j++) {
                myfile118 << all_BestBT[b][j].child_num;
                if (j!=all_BestBT[b].size()-1)
                {
                    myfile118<< ",";
                }
            }
            myfile118 << "\n";
        }
        myfile118.close();

        std::ofstream myfile30;
        myfile30.open (sub_folder+Worst_BTs_"+to_string((i))+".csv");
        for (int b=0;b<generations_number;b++)
        {
            for ( int j=0; j<all_worstBT[b].size(); j++) {
                myfile30 << all_worstBT[b][j].node;
                if (j!=all_worstBT[b].size()-1)
                {
                    myfile30<< ",";
                }

            }
            myfile30 << "\n";
            for ( int j=0; j<all_worstBT[b].size(); j++) {
                myfile30 << all_worstBT[b][j].child_num;
                if (j!=all_worstBT[b].size()-1)
                {
                    myfile30<< ",";
                }
            }
            myfile30 << "\n";
        }
        myfile30.close();

        std::ofstream myfile19;
        myfile19.open (sub_folder+BestFitness"+to_string((i))+".csv");

        for(int i=0; i<Best_Fitness.size(); i++)
        {            myfile19 << Best_Fitness[i] << ",";
        }
        myfile19 << "\n";
        myfile19.close();

        std::ofstream myfile20;
        myfile20.open (sub_folder+Avg_Fitness"+to_string((i))+".csv");

        for(int i=0; i<Avg_Fitness.size(); i++)
        {            myfile20 << Avg_Fitness[i] << ",";
        }
        myfile20 << "\n";
        myfile20.close();

        std::ofstream myfile40;
        myfile40.open (sub_folder+Max_Fitness"+to_string((i))+".csv");

        for(int i=0; i<Max_Fitness.size(); i++)
        {            myfile40 << Max_Fitness[i] << ",";
        }
        myfile40 << "\n";
        myfile40.close();

        std::ofstream myfile21;
        myfile21.open (sub_folder+BestBT_Metrics"+std::to_string(i)+".csv");
        for (int i=0; i< blackboard::simulation_timesteps ;i++)
        {
            for ( int j=0; j<BestBT_metrics.size(); j++){
                myfile21 << BestBT_metrics[j][i]<< ","  ;
            }
            myfile21 <<"\n"  ;
        }
        myfile21.close();

        std::ofstream myfile22;
        myfile22.open (sub_folder+OriginalBT_Metrics"+std::to_string(i)+".csv");

        for (int i=0; i< blackboard::simulation_timesteps ;i++)
        {
            for ( int j=0; j<original_metrics.size(); j++){
                myfile22 << original_metrics[j][i]<< ",";
            }
            myfile22 << "\n"  ;
        }
        myfile22.close();

        std::ofstream myfile23;
        myfile23.open (sub_folder+All_BTs"+std::to_string(i)+".csv");

        for (int i=0; i< all_BT.size() ;i++)
        {
            for ( int j=0; j<all_BT[i].size(); j++){

                for ( int k=0; k<all_BT[i][j].size(); k++){

                    myfile23 << all_BT[i][j][k].node<< ",";
                }
                myfile23 << "\n";

                for ( int k=0; k<all_BT[i][j].size(); k++){

                    myfile23 << all_BT[i][j][k].child_num<< ",";
                }
                myfile23 << "\n";
            }
        }
        myfile23.close();

        std::ofstream myfile24;
        myfile24.open(sub_folder+Best_Metrics_Fitness" +
                      std::to_string(i) + ".csv");

        for (int i = 0; i < best_multi_fit.size(); i++) {
            for (int j = 0; j < best_multi_fit[i].size(); j++) {

                myfile24 << best_multi_fit[i][j] << ",";
            }
            myfile24 << "\n";
        }
        myfile24.close();

        std::ofstream myfile25;
        myfile25.open(sub_folder+Avg_Metrics_Fitness_" +
                      std::to_string(i) + ".csv");

        for (int i = 0; i < avg_multi_fit.size(); i++) {
            for (int j = 0; j < avg_multi_fit[i].size(); j++) {

                myfile25 << avg_multi_fit[i][j] << ",";
            }
            myfile25 << "\n";
        }

        myfile25.close();

        std::ofstream myfile26;
        myfile26.open(sub_folder+Worst_Metrics_Fitness_" +
                      std::to_string(i) + ".csv");

        for (int i = 0; i < max_multi_fit.size(); i++) {
            for (int j = 0; j < max_multi_fit[i].size(); j++) {

                myfile26 << max_multi_fit[i][j] << ",";
            }
            myfile26 << "\n";
        }

        myfile26.close();

        if (MA_data.size()>0)
        {
            std::ofstream myfile27;
            myfile27.open(sub_folder+local_search_data_" +
                          std::to_string(i) + ".csv");

            for (int d = 0; d < MA_data.size(); d++) {
                myfile27 <<get<0>(MA_data[d]) << ",";
                myfile27 <<get<1>(MA_data[d]) << ",";
                myfile27 <<get<2>(MA_data[d]) << ",";
                myfile27 <<get<3>(MA_data[d]) << ",";
                myfile27 <<get<4>(MA_data[d]);
                myfile27 << "\n";
            }

            myfile27.close();
        }
    }

    void save_final_sim_data_agent_level (int i,vector<blackboard::Node>  BTree)
    {
        // Save original BT data
        std::ofstream myfile;
        myfile.open (sub_folder+Original_SwarmX"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<blackboard::Swarm_size; i++)
            {            myfile << original.Sim1.SwarmX[j*blackboard::Swarm_size+i];
                if (i!=blackboard::Swarm_size-1)
                {
                    myfile<< ",";
                }
            }
            myfile << "\n";
        }
        myfile.close();

        std::ofstream myfile2;
        myfile2.open (sub_folder+Original_SwarmY"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<blackboard::Swarm_size; i++)
            {
                myfile2 << original.Sim1.SwarmY[j*blackboard::Swarm_size+i] ;
                if (i!=blackboard::Swarm_size-1)
                {
                    myfile2<< ",";
                }
            }
            myfile2 << "\n";
        }
        myfile2.close();




        std::ofstream myfile3;
        myfile3.open (sub_folder+Original_SwarmEnv"+to_string((i))+".csv");
        if(original.Sim1.controller.blackboard.BT_Env.areas_in_env)
        {
            myfile3 << 1 << "," ;
            std::ofstream myfile4;
            myfile4.open (sub_folder+Original_SwarmEnvAreasX"+to_string((i))+".csv");
            std::ofstream myfile5;
            myfile5.open (sub_folder+Original_SwarmEnvAreasY"+to_string((i))+".csv");
            std::ofstream myfile6;
            myfile6.open (sub_folder+Original_SwarmEnvAreasWidth"+to_string((i))+".csv");
            for(int i=0; i<original.Sim1.controller.blackboard.BT_Env.areas_num; i++)
            {
                myfile4 << original.Sim1.controller.blackboard.BT_Env.areas_X[i]<< "," ;
                myfile5 << original.Sim1.controller.blackboard.BT_Env.areas_Y[i]<< "," ;
                myfile6 << original.Sim1.controller.blackboard.BT_Env.areas_Width[i]<< "," ;
            }
            myfile4.close();
            myfile5.close();
            myfile6.close();

        }
        else
            myfile3 << 0 << "," ;
        if(original.Sim1.controller.blackboard.BT_Env.objects_in_env)
        {
            myfile3 << 1 << "," ;

            std::ofstream myfile4;
            myfile4.open (sub_folder+Original_SwarmEnvObjectX"+to_string((i))+".csv");
            std::ofstream myfile5;
            myfile5.open (sub_folder+Original_SwarmEnvObjectY"+to_string((i))+".csv");
            for ( int i=0; i<blackboard::simulation_timesteps; i++){
                for(int j=0; j<original.Sim1.controller.blackboard.BT_Env.objects_num; j++)
                {
                    myfile4 << original.Sim1.objects_Data_X[i][j]<< "," ;
                    myfile5 << original.Sim1.objects_Data_Y[i][j]<< "," ;

                }
                myfile4 << "\n";
                myfile5 << "\n";

            }
            myfile4.close();
            myfile5.close();


        }
        else
            myfile3 << 0 << "," ;
        if(original.Sim1.controller.blackboard.BT_Env.obstacles_in_env)
        {
            myfile3 << 1 << "," ;
            std::ofstream myfile4;
            myfile4.open (sub_folder+Original_SwarmEnvObstaclesX"+to_string((i))+".csv");
            std::ofstream myfile5;
            myfile5.open (sub_folder+Original_SwarmEnvObstaclesY"+to_string((i))+".csv");
            std::ofstream myfile6;
            myfile6.open (sub_folder+Original_SwarmEnvObstaclesX2"+to_string((i))+".csv");
            std::ofstream myfile7;
            myfile7.open (sub_folder+Original_SwarmEnvObstaclesY2"+to_string((i))+".csv");
            for(int i=0; i<original.Sim1.controller.blackboard.BT_Env.obstacles_num; i++)
            {
                myfile4 << original.Sim1.controller.blackboard.BT_Env.obstacles_X[i]<< "," ;
                myfile5 << original.Sim1.controller.blackboard.BT_Env.obstacles_Y[i]<< "," ;
                myfile6 << original.Sim1.controller.blackboard.BT_Env.obstacles_X2[i]<< "," ;
                myfile7 << original.Sim1.controller.blackboard.BT_Env.obstacles_Y2[i]<< "," ;

            }
            myfile4.close();
            myfile5.close();
            myfile6.close();
            myfile7.close();


        }
        else
            myfile3 << 0 << "," ;
        myfile3.close();

        std::ofstream myfile8;
        myfile8.open (sub_folder+Original_BT"+to_string((i))+".csv");
        for ( int j=0; j<original.Sim1.controller.BT.size(); j++) {
            myfile8 << original.Sim1.controller.BT[j].node << ",";
        }
        myfile8 << "\n";
        for ( int j=0; j<original.Sim1.controller.BT.size(); j++) {
            myfile8 << original.Sim1.controller.BT[j].child_num << ",";
        }
        myfile8.close();


        // Save imitated BT data
        std::ofstream myfile9;
        myfile9.open (sub_folder+imitated_SwarmX"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<blackboard::Swarm_size; i++)
            {            myfile9 << get<1>(swarm_trajectories[0])[j][i]<< "," ;
            }
            myfile9 << "\n";
        }
        myfile9.close();


        std::ofstream myfile12;
        myfile12.open (sub_folder+imitated_SwarmY"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<blackboard::Swarm_size; i++)
            {
                myfile12 << get<2>(swarm_trajectories[0])[j][i]<< "," ;
            }
            myfile12 << "\n";
        }
        myfile12.close();



        std::ofstream myfile13;
        myfile13.open (sub_folder+imitated_SwarmEnv"+to_string((i))+".csv");
        if(get<2>(population_Fit_Env[0]).areas_in_env)
        {
            myfile13 << 1 << "," ;
            std::ofstream myfile14;
            myfile14.open (sub_folder+imitated_SwarmEnvAreasX"+to_string((i))+".csv");
            std::ofstream myfile15;
            myfile15.open (sub_folder+imitated_SwarmEnvAreasY"+to_string((i))+".csv");
            std::ofstream myfile16;
            myfile16.open (sub_folder+imitated_SwarmEnvAreasWidth"+to_string((i))+".csv");
            for(int i=0; i<get<2>(population_Fit_Env[0]).areas_num; i++)
            {
                myfile14 <<get<2>(population_Fit_Env[0]).areas_X[i]<< "," ;
                myfile15 << get<2>(population_Fit_Env[0]).areas_Y[i]<< "," ;
                myfile16 << get<2>(population_Fit_Env[0]).areas_Width[i]<< "," ;
            }
            myfile14.close();
            myfile15.close();
            myfile16.close();
        }
        else
            myfile13 << 0 << "," ;
        if(get<2>(population_Fit_Env[0]).objects_in_env)
        {
            myfile13 << 1 << "," ;

            std::ofstream myfile14;
            myfile14.open (sub_folder+imitated_SwarmEnvObjectX"+to_string((i))+".csv");
            std::ofstream myfile15;
            myfile15.open (sub_folder+imitated_SwarmEnvObjectY"+to_string((i))+".csv");
            for ( int i=0; i<blackboard::simulation_timesteps; i++){
                for(int j=0; j<get<2>(population_Fit_Env[0]).objects_num; j++)
                {
                    myfile14 << get<2>(population_Fit_Env[0]).objects_Data_X[i][j]<< "," ;
                    myfile15 << get<2>(population_Fit_Env[0]).objects_Data_Y[i][j]<< "," ;

                }
                myfile14 << "\n";
                myfile15 << "\n";

            }
            myfile14.close();
            myfile15.close();

        }
        else
            myfile13 << 0 << "," ;
        if(get<2>(population_Fit_Env[0]).obstacles_in_env)
        {
            myfile13 << 1 << "," ;
            std::ofstream myfile14;
            myfile14.open (sub_folder+imitated_SwarmEnvObstaclesX"+to_string((i))+".csv");
            std::ofstream myfile15;
            myfile15.open (sub_folder+imitated_SwarmEnvObstaclesY"+to_string((i))+".csv");
            std::ofstream myfile16;
            myfile16.open (sub_folder+imitated_SwarmEnvObstaclesX2"+to_string((i))+".csv");
            std::ofstream myfile17;
            myfile17.open (sub_folder+imitated_SwarmEnvObstaclesY2"+to_string((i))+".csv");
            for(int i=0; i<get<2>(population_Fit_Env[0]).obstacles_num; i++)
            {
                myfile14 << get<2>(population_Fit_Env[0]).obstacles_X[i]<< "," ;
                myfile15 << get<2>(population_Fit_Env[0]).obstacles_Y[i]<< "," ;
                myfile16 << get<2>(population_Fit_Env[0]).obstacles_X2[i]<< "," ;
                myfile17 << get<2>(population_Fit_Env[0]).obstacles_Y2[i]<< "," ;

            }
            myfile14.close();
            myfile15.close();
            myfile16.close();
            myfile17.close();

        }
        else
            myfile13 << 0 << "," ;
        myfile13.close();
        std::ofstream myfile18;
        myfile18.open (sub_folder+imitated_BT"+to_string((i))+".csv");
        for ( int j=0; j<BTree.size(); j++) {
            myfile18 << BTree[j].node << ",";

        }
        myfile18 << "\n";
        for ( int j=0; j<BTree.size(); j++) {
            myfile18 << BTree[j].child_num << ",";
        }
        myfile18.close();

        std::ofstream myfile118;
        myfile118.open (sub_folder+Best_BTs_"+to_string((i))+".csv");
        for (int b=0;b<generations_number;b++)
        {
            for ( int j=0; j<all_BestBT[b].size(); j++) {
                myfile118 << all_BestBT[b][j].node;
                if (j!=all_BestBT[b].size()-1)
                {
                    myfile118<< ",";
                }

            }
            myfile118 << "\n";
            for ( int j=0; j<all_BestBT[b].size(); j++) {
                myfile118 << all_BestBT[b][j].child_num;
                if (j!=all_BestBT[b].size()-1)
                {
                    myfile118<< ",";
                }
            }
            myfile118 << "\n";
        }
        myfile118.close();

        std::ofstream myfile30;
        myfile30.open (sub_folder+Worst_BTs_"+to_string((i))+".csv");
        for (int b=0;b<generations_number;b++)
        {
            for ( int j=0; j<all_worstBT[b].size(); j++) {
                myfile30 << all_worstBT[b][j].node;
                if (j!=all_worstBT[b].size()-1)
                {
                    myfile30<< ",";
                }

            }
            myfile30 << "\n";
            for ( int j=0; j<all_worstBT[b].size(); j++) {
                myfile30 << all_worstBT[b][j].child_num;
                if (j!=all_worstBT[b].size()-1)
                {
                    myfile30<< ",";
                }
            }
            myfile30 << "\n";
        }
        myfile30.close();

        std::ofstream myfile19;
        myfile19.open (sub_folder+BestFitness"+to_string((i))+".csv");

        for(int i=0; i<Best_Fitness.size(); i++)
        {            myfile19 << Best_Fitness[i] << ",";
        }
        myfile19 << "\n";
        myfile19.close();

        std::ofstream myfile20;
        myfile20.open (sub_folder+Avg_Fitness"+to_string((i))+".csv");

        for(int i=0; i<Avg_Fitness.size(); i++)
        {            myfile20 << Avg_Fitness[i] << ",";
        }
        myfile20 << "\n";
        myfile20.close();

        std::ofstream myfile40;
        myfile40.open (sub_folder+Max_Fitness"+to_string((i))+".csv");

        for(int i=0; i<Max_Fitness.size(); i++)
        {            myfile40 << Max_Fitness[i] << ",";
        }
        myfile40 << "\n";
        myfile40.close();

        std::ofstream myfile21;
        myfile21.open (sub_folder+BestBT_Metrics"+std::to_string(i)+".csv");
        for (int i=0; i< blackboard::simulation_timesteps ;i++)
        {
            for ( int j=0; j<BestBT_metrics.size(); j++){
                myfile21 << BestBT_metrics[j][i]<< ","  ;
            }
            myfile21 <<"\n"  ;
        }
        myfile21.close();

        for (int m=0 ; m<BestBT_metrics_agents.size();m++)
        {
            std::ofstream myfile21;
            myfile21.open (sub_folder+BestBT_Metrics_no"+std::to_string(m)+"_"+std::to_string(i)+".csv");
            for (int i=0; i< blackboard::simulation_timesteps ;i++)
            {
                for ( int j=0; j<BestBT_metrics_agents[m].size(); j++){
                    myfile21 << BestBT_metrics_agents[m][j][i]<< ","  ;
                }
                myfile21 <<"\n"  ;
            }
            myfile21.close();

            std::ofstream myfile22;
            myfile22.open (sub_folder+OriginalBT_Metrics_no"+std::to_string(m)+"_"+std::to_string(i)+".csv");

            for (int i=0; i< blackboard::simulation_timesteps ;i++)
            {
                for ( int j=0; j<originals_metrics[m].size(); j++){
                    myfile22 << originals_metrics[m][j][i]<< ",";
                }
                myfile22 << "\n"  ;
            }
            myfile22.close();
        }



        std::ofstream myfile23;
        myfile23.open (sub_folder+All_BTs"+std::to_string(i)+".csv");

        for (int i=0; i< all_BT.size() ;i++)
        {
            for ( int j=0; j<all_BT[i].size(); j++){

                for ( int k=0; k<all_BT[i][j].size(); k++){

                    myfile23 << all_BT[i][j][k].node<< ",";
                }
                myfile23 << "\n";

                for ( int k=0; k<all_BT[i][j].size(); k++){

                    myfile23 << all_BT[i][j][k].child_num<< ",";
                }
                myfile23 << "\n";
            }
        }
        myfile23.close();

        std::ofstream myfile24;
        myfile24.open(sub_folder+Best_Metrics_Fitness" +
                      std::to_string(i) + ".csv");

        for (int i = 0; i < best_multi_fit.size(); i++) {
            for (int j = 0; j < best_multi_fit[i].size(); j++) {

                myfile24 << best_multi_fit[i][j] << ",";
            }
            myfile24 << "\n";
        }
        myfile24.close();

/*        std::ofstream myfile25;
        myfile25.open(sub_folder+Avg_Metrics_Fitness_" +
                      std::to_string(i) + ".csv");

        for (int i = 0; i < avg_multi_fit.size(); i++) {
            for (int j = 0; j < avg_multi_fit[i].size(); j++) {

                myfile25 << avg_multi_fit[i][j] << ",";
            }
            myfile25 << "\n";
        }

        myfile25.close();*/

        std::ofstream myfile26;
        myfile26.open(sub_folder+Worst_Metrics_Fitness_" +
                      std::to_string(i) + ".csv");

        for (int i = 0; i < max_multi_fit.size(); i++) {
            for (int j = 0; j < max_multi_fit[i].size(); j++) {

                myfile26 << max_multi_fit[i][j] << ",";
            }
            myfile26 << "\n";
        }

        myfile26.close();
    }

    void save_final_vid_data (int i,vector<blackboard::Node>  BTree)
    {
        // Save original BT data
        std::ofstream myfile;
        myfile.open (sub_folder+Original_SwarmX"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<blackboard::Swarm_size; i++)
            {            myfile << original.Sim1.SwarmX[j*blackboard::Swarm_size+i];
                if (i!=blackboard::Swarm_size-1)
                {
                    myfile<< ",";
                }
            }
            myfile << "\n";
        }
        myfile.close();

        std::ofstream myfile2;
        myfile2.open (sub_folder+Original_SwarmY"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<blackboard::Swarm_size; i++)
            {
                myfile2 << original.Sim1.SwarmY[j*blackboard::Swarm_size+i] ;
                if (i!=blackboard::Swarm_size-1)
                {
                    myfile2<< ",";
                }
            }
            myfile2 << "\n";
        }
        myfile2.close();


        std::ofstream myfile3;
        myfile3.open (sub_folder+Original_SwarmEnv"+to_string((i))+".csv");
        if(original.Sim1.controller.blackboard.BT_Env.areas_in_env)
        {
            myfile3 << 1 << "," ;
            std::ofstream myfile4;
            myfile4.open (sub_folder+Original_SwarmEnvAreasX"+to_string((i))+".csv");
            std::ofstream myfile5;
            myfile5.open (sub_folder+Original_SwarmEnvAreasY"+to_string((i))+".csv");
            std::ofstream myfile6;
            myfile6.open (sub_folder+Original_SwarmEnvAreasWidth"+to_string((i))+".csv");
            for(int i=0; i<original.Sim1.controller.blackboard.BT_Env.areas_num; i++)
            {
                myfile4 << original.Sim1.controller.blackboard.BT_Env.areas_X[i]<< "," ;
                myfile5 << original.Sim1.controller.blackboard.BT_Env.areas_Y[i]<< "," ;
                myfile6 << original.Sim1.controller.blackboard.BT_Env.areas_Width[i]<< "," ;
            }
            myfile4.close();
            myfile5.close();
            myfile6.close();
        }
        else
            myfile3 << 0 << "," ;
        if(original.Sim1.controller.blackboard.BT_Env.objects_in_env)
        {
            myfile3 << 1 << "," ;

            std::ofstream myfile4;
            myfile4.open (sub_folder+Original_SwarmEnvObjectX"+to_string((i))+".csv");
            std::ofstream myfile5;
            myfile5.open (sub_folder+Original_SwarmEnvObjectY"+to_string((i))+".csv");
            for ( int i=0; i<blackboard::simulation_timesteps; i++){
                for(int j=0; j<original.Sim1.controller.blackboard.BT_Env.objects_num; j++)
                {
                    myfile4 << original.Sim1.objects_Data_X[i][j]<< "," ;
                    myfile5 << original.Sim1.objects_Data_Y[i][j]<< "," ;

                }
                myfile4 << "\n";
                myfile5 << "\n";

            }
            myfile4.close();
            myfile5.close();

        }
        else
            myfile3 << 0 << "," ;
        if(original.Sim1.controller.blackboard.BT_Env.obstacles_in_env)
        {
            myfile3 << 1 << "," ;
            std::ofstream myfile4;
            myfile4.open (sub_folder+Original_SwarmEnvObstaclesX"+to_string((i))+".csv");
            std::ofstream myfile5;
            myfile5.open (sub_folder+Original_SwarmEnvObstaclesY"+to_string((i))+".csv");
            std::ofstream myfile6;
            myfile6.open (sub_folder+Original_SwarmEnvObstaclesX2"+to_string((i))+".csv");
            std::ofstream myfile7;
            myfile7.open (sub_folder+Original_SwarmEnvObstaclesY2"+to_string((i))+".csv");
            for(int i=0; i<original.Sim1.controller.blackboard.BT_Env.obstacles_num; i++)
            {
                myfile4 << original.Sim1.controller.blackboard.BT_Env.obstacles_X[i]<< "," ;
                myfile5 << original.Sim1.controller.blackboard.BT_Env.obstacles_Y[i]<< "," ;
                myfile6 << original.Sim1.controller.blackboard.BT_Env.obstacles_X2[i]<< "," ;
                myfile7 << original.Sim1.controller.blackboard.BT_Env.obstacles_Y2[i]<< "," ;

            }
            myfile4.close();
            myfile5.close();
            myfile6.close();
            myfile7.close();

        }
        else
            myfile3 << 0 << "," ;
        myfile3.close();

        std::ofstream myfile8;
        myfile8.open (sub_folder+Original_BT"+to_string((i))+".csv");

        myfile8 << 0<< ","<<2<< ",";

        myfile8 << "\n";
        myfile8 << 1<< ","<<0<< ",";

        myfile8.close();




        // Save imitated BT data
        std::ofstream myfile9;
        myfile9.open (sub_folder+imitated_SwarmX"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<blackboard::Swarm_size; i++)
            {            myfile9 << get<1>(swarm_trajectories[0])[j][i]<< "," ;
            }
            myfile9 << "\n";
        }
        myfile9.close();


        std::ofstream myfile12;
        myfile12.open (sub_folder+imitated_SwarmY"+to_string((i))+".csv");
        for ( int j=0; j<blackboard::simulation_timesteps; j++){
            for(int i=0; i<blackboard::Swarm_size; i++)
            {
                myfile12 << get<2>(swarm_trajectories[0])[j][i]<< "," ;
            }
            myfile12 << "\n";
        }
        myfile12.close();



        std::ofstream myfile13;
        myfile13.open (sub_folder+imitated_SwarmEnv"+to_string((i))+".csv");
        if(get<2>(population_Fit_Env[0]).areas_in_env)
        {
            myfile13 << 1 << "," ;
            std::ofstream myfile14;
            myfile14.open (sub_folder+imitated_SwarmEnvAreasX"+to_string((i))+".csv");
            std::ofstream myfile15;
            myfile15.open (sub_folder+imitated_SwarmEnvAreasY"+to_string((i))+".csv");
            std::ofstream myfile16;
            myfile16.open (sub_folder+imitated_SwarmEnvAreasWidth"+to_string((i))+".csv");
            for(int i=0; i<get<2>(population_Fit_Env[0]).areas_num; i++)
            {
                myfile14 <<get<2>(population_Fit_Env[0]).areas_X[i]<< "," ;
                myfile15 << get<2>(population_Fit_Env[0]).areas_Y[i]<< "," ;
                myfile16 << get<2>(population_Fit_Env[0]).areas_Width[i]<< "," ;
            }
            myfile14.close();
            myfile15.close();
            myfile16.close();
        }
        else
            myfile13 << 0 << "," ;
        if(get<2>(population_Fit_Env[0]).objects_in_env)
        {
            myfile13 << 1 << "," ;

            std::ofstream myfile14;
            myfile14.open (sub_folder+imitated_SwarmEnvObjectX"+to_string((i))+".csv");
            std::ofstream myfile15;
            myfile15.open (sub_folder+imitated_SwarmEnvObjectY"+to_string((i))+".csv");
            for ( int i=0; i<blackboard::simulation_timesteps; i++){
                for(int j=0; j<get<2>(population_Fit_Env[0]).objects_num; j++)
                {
                    myfile14 << get<2>(population_Fit_Env[0]).objects_Data_X[i][j]<< "," ;
                    myfile15 << get<2>(population_Fit_Env[0]).objects_Data_Y[i][j]<< "," ;

                }
                myfile14 << "\n";
                myfile15 << "\n";

            }
            myfile14.close();
            myfile15.close();

        }
        else
            myfile13 << 0 << "," ;
        if(get<2>(population_Fit_Env[0]).obstacles_in_env)
        {
            myfile13 << 1 << "," ;
            std::ofstream myfile14;
            myfile14.open (sub_folder+imitated_SwarmEnvObstaclesX"+to_string((i))+".csv");
            std::ofstream myfile15;
            myfile15.open (sub_folder+imitated_SwarmEnvObstaclesY"+to_string((i))+".csv");
            std::ofstream myfile16;
            myfile16.open (sub_folder+imitated_SwarmEnvObstaclesX2"+to_string((i))+".csv");
            std::ofstream myfile17;
            myfile17.open (sub_folder+imitated_SwarmEnvObstaclesY2"+to_string((i))+".csv");
            for(int i=0; i<get<2>(population_Fit_Env[0]).obstacles_num; i++)
            {
                myfile14 << get<2>(population_Fit_Env[0]).obstacles_X[i]<< "," ;
                myfile15 << get<2>(population_Fit_Env[0]).obstacles_Y[i]<< "," ;
                myfile16 << get<2>(population_Fit_Env[0]).obstacles_X2[i]<< "," ;
                myfile17 << get<2>(population_Fit_Env[0]).obstacles_Y2[i]<< "," ;

            }
            myfile14.close();
            myfile15.close();
            myfile16.close();
            myfile17.close();

        }
        else
            myfile13 << 0 << "," ;
        myfile13.close();
        std::ofstream myfile18;
        myfile18.open (sub_folder+imitated_BT"+to_string((i))+".csv");
        for ( int j=0; j<BTree.size(); j++) {
            myfile18 << BTree[j].node << ",";

        }
        myfile18 << "\n";
        for ( int j=0; j<BTree.size(); j++) {
            myfile18 << BTree[j].child_num << ",";
        }
        myfile18.close();

        std::ofstream myfile118;
        myfile118.open (sub_folder+Best_BTs_"+to_string((i))+".csv");
        for (int b=0;b<generations_number;b++)
        {
            for ( int j=0; j<all_BestBT[b].size(); j++) {
                myfile118 << all_BestBT[b][j].node;
                if (j!=all_BestBT[b].size()-1)
                {
                    myfile118<< ",";
                }

            }
            myfile118 << "\n";
            for ( int j=0; j<all_BestBT[b].size(); j++) {
                myfile118 << all_BestBT[b][j].child_num;
                if (j!=all_BestBT[b].size()-1)
                {
                    myfile118<< ",";
                }
            }
            myfile118 << "\n";
        }
        myfile118.close();

        std::ofstream myfile30;
        myfile30.open (sub_folder+Worst_BTs_"+to_string((i))+".csv");
        for (int b=0;b<generations_number;b++)
        {
            for ( int j=0; j<all_worstBT[b].size(); j++) {
                myfile30 << all_worstBT[b][j].node;
                if (j!=all_worstBT[b].size()-1)
                {
                    myfile30<< ",";
                }

            }
            myfile30 << "\n";
            for ( int j=0; j<all_worstBT[b].size(); j++) {
                myfile30 << all_worstBT[b][j].child_num;
                if (j!=all_worstBT[b].size()-1)
                {
                    myfile30<< ",";
                }
            }
            myfile30 << "\n";
        }
        myfile30.close();

        std::ofstream myfile19;
        myfile19.open (sub_folder+BestFitness"+to_string((i))+".csv");

        for(int i=0; i<Best_Fitness.size(); i++)
        {            myfile19 << Best_Fitness[i] << ",";
        }
        myfile19 << "\n";
        myfile19.close();

        std::ofstream myfile20;
        myfile20.open (sub_folder+Avg_Fitness"+to_string((i))+".csv");

        for(int i=0; i<Avg_Fitness.size(); i++)
        {            myfile20 << Avg_Fitness[i] << ",";
        }
        myfile20 << "\n";
        myfile20.close();

        std::ofstream myfile40;
        myfile40.open (sub_folder+Max_Fitness"+to_string((i))+".csv");

        for(int i=0; i<Max_Fitness.size(); i++)
        {            myfile40 << Max_Fitness[i] << ",";
        }
        myfile40 << "\n";
        myfile40.close();

        std::ofstream myfile21;
        myfile21.open (sub_folder+BestBT_Metrics"+std::to_string(i)+".csv");
        for (int i=0; i< blackboard::simulation_timesteps ;i++)
        {
            for ( int j=0; j<BestBT_metrics.size(); j++){
                myfile21 << BestBT_metrics[j][i]<< ","  ;
            }
            myfile21 <<"\n"  ;
        }
        myfile21.close();

        std::ofstream myfile22;
        myfile22.open (sub_folder+OriginalBT_Metrics"+std::to_string(i)+".csv");

        for (int i=0; i< blackboard::simulation_timesteps ;i++)
        {
            for ( int j=0; j<original_metrics.size(); j++){
                myfile22 << original_metrics[j][i]<< ",";
            }
            myfile22 << "\n"  ;
        }
        myfile22.close();

        std::ofstream myfile23;
        myfile23.open (sub_folder+All_BTs"+std::to_string(i)+".csv");

        for (int i=0; i< all_BT.size() ;i++)
        {
            for ( int j=0; j<all_BT[i].size(); j++){

                for ( int k=0; k<all_BT[i][j].size(); k++){

                    myfile23 << all_BT[i][j][k].node<< ",";
                }
                myfile23 << "\n";

                for ( int k=0; k<all_BT[i][j].size(); k++){

                    myfile23 << all_BT[i][j][k].child_num<< ",";
                }
                myfile23 << "\n";
            }
        }
        myfile23.close();



    }

    void save_final_sim_data (int i,vector<blackboard::Node>  BTree,bool multiple_demos) {
        // Save original BT data
        std::ofstream myfile;
        myfile.open(sub_folder+Original_SwarmX" +
                    to_string((i)) + ".csv");
        for (int j = 0; j < blackboard::simulation_timesteps; j++) {
            for (int i = 0; i < blackboard::Swarm_size; i++) {
                myfile << originals[0].Sim1.SwarmX[j * blackboard::Swarm_size + i];
                if (i != blackboard::Swarm_size - 1) {
                    myfile << ",";
                }
            }
            myfile << "\n";
        }
        myfile.close();

        std::ofstream myfile2;
        myfile2.open(sub_folder+Original_SwarmY" +
                     to_string((i)) + ".csv");
        for (int j = 0; j < blackboard::simulation_timesteps; j++) {
            for (int i = 0; i < blackboard::Swarm_size; i++) {
                myfile2 << originals[0].Sim1.SwarmY[j * blackboard::Swarm_size + i];
                if (i != blackboard::Swarm_size - 1) {
                    myfile2 << ",";
                }
            }
            myfile2 << "\n";
        }
        myfile2.close();


        std::ofstream myfile3;
        myfile3.open(sub_folder+Original_SwarmEnv" +
                     to_string((i)) + ".csv");
        if (originals[0].Sim1.controller.blackboard.BT_Env.areas_in_env) {
            myfile3 << 1 << ",";
            std::ofstream myfile4;
            myfile4.open(
                    sub_folder+Original_SwarmEnvAreasX" +
                    to_string((i)) + ".csv");
            std::ofstream myfile5;
            myfile5.open(
                    sub_folder+Original_SwarmEnvAreasY" +
                    to_string((i)) + ".csv");
            std::ofstream myfile6;
            myfile6.open(
                    sub_folder+Original_SwarmEnvAreasWidth" +
                    to_string((i)) + ".csv");
            for (int i = 0; i < originals[0].Sim1.controller.blackboard.BT_Env.areas_num; i++) {
                myfile4 << originals[0].Sim1.controller.blackboard.BT_Env.areas_X[i] << ",";
                myfile5 << originals[0].Sim1.controller.blackboard.BT_Env.areas_Y[i] << ",";
                myfile6 << originals[0].Sim1.controller.blackboard.BT_Env.areas_Width[i] << ",";
            }
            myfile4.close();
            myfile5.close();
            myfile6.close();
        } else
            myfile3 << 0 << ",";
        if (originals[0].Sim1.controller.blackboard.BT_Env.objects_in_env) {
            myfile3 << 1 << ",";

            std::ofstream myfile4;
            myfile4.open(
                    sub_folder+Original_SwarmEnvObjectX" +
                    to_string((i)) + ".csv");
            std::ofstream myfile5;
            myfile5.open(
                    sub_folder+Original_SwarmEnvObjectY" +
                    to_string((i)) + ".csv");
            for (int i = 0; i < blackboard::simulation_timesteps; i++) {
                for (int j = 0; j < originals[0].Sim1.controller.blackboard.BT_Env.objects_num; j++) {
                    myfile4 << originals[0].Sim1.controller.blackboard.BT_Env.objects_Data_X[i][j] << ",";
                    myfile5 << originals[0].Sim1.controller.blackboard.BT_Env.objects_Data_Y[i][j] << ",";

                }
                myfile4 << "\n";
                myfile5 << "\n";

            }
            myfile4.close();
            myfile5.close();

        } else
            myfile3 << 0 << ",";
        if (originals[0].Sim1.controller.blackboard.BT_Env.obstacles_in_env) {
            myfile3 << 1 << ",";
            std::ofstream myfile4;
            myfile4.open(
                    sub_folder+Original_SwarmEnvObstaclesX" +
                    to_string((i)) + ".csv");
            std::ofstream myfile5;
            myfile5.open(
                    sub_folder+Original_SwarmEnvObstaclesY" +
                    to_string((i)) + ".csv");
            std::ofstream myfile6;
            myfile6.open(
                    sub_folder+Original_SwarmEnvObstaclesX2" +
                    to_string((i)) + ".csv");
            std::ofstream myfile7;
            myfile7.open(
                    sub_folder+Original_SwarmEnvObstaclesY2" +
                    to_string((i)) + ".csv");
            for (int i = 0; i < originals[0].Sim1.controller.blackboard.BT_Env.obstacles_num; i++) {
                myfile4 << originals[0].Sim1.controller.blackboard.BT_Env.obstacles_X[i] << ",";
                myfile5 << originals[0].Sim1.controller.blackboard.BT_Env.obstacles_Y[i] << ",";
                myfile6 << originals[0].Sim1.controller.blackboard.BT_Env.obstacles_X2[i] << ",";
                myfile7 << originals[0].Sim1.controller.blackboard.BT_Env.obstacles_Y2[i] << ",";

            }
            myfile4.close();
            myfile5.close();
            myfile6.close();
            myfile7.close();

        } else
            myfile3 << 0 << ",";
        myfile3.close();

        std::ofstream myfile8;
        myfile8.open(
                sub_folder+Original_BT" + to_string((i)) +
                ".csv");
        for (int j = 0; j < originals[0].Sim1.controller.BT.size(); j++) {
            myfile8 << originals[0].Sim1.controller.BT[j].node << ",";
        }
        myfile8 << "\n";
        for (int j = 0; j < originals[0].Sim1.controller.BT.size(); j++) {
            myfile8 << originals[0].Sim1.controller.BT[j].child_num << ",";
        }
        myfile8.close();




        // Save imitated BT data
        std::ofstream myfile9;
        myfile9.open(sub_folder+imitated_SwarmX" +
                     to_string((i)) + ".csv");
        for (int j = 0; j < blackboard::simulation_timesteps; j++) {
            for (int i = 0; i < blackboard::Swarm_size; i++) {
                myfile9 << get<1>(swarm_trajectories[0])[j][i] << ",";
            }
            myfile9 << "\n";
        }
        myfile9.close();


        std::ofstream myfile12;
        myfile12.open(sub_folder+imitated_SwarmY" +
                      to_string((i)) + ".csv");
        for (int j = 0; j < blackboard::simulation_timesteps; j++) {
            for (int i = 0; i < blackboard::Swarm_size; i++) {
                myfile12 << get<2>(swarm_trajectories[0])[j][i] << ",";
            }
            myfile12 << "\n";
        }
        myfile12.close();


        std::ofstream myfile13;
        myfile13.open(sub_folder+imitated_SwarmEnv" +
                      to_string((i)) + ".csv");
        if (get<2>(population_Fit_Env[0]).areas_in_env) {
            myfile13 << 1 << ",";
            std::ofstream myfile14;
            myfile14.open(
                    sub_folder+imitated_SwarmEnvAreasX" +
                    to_string((i)) + ".csv");
            std::ofstream myfile15;
            myfile15.open(
                    sub_folder+imitated_SwarmEnvAreasY" +
                    to_string((i)) + ".csv");
            std::ofstream myfile16;
            myfile16.open(
                    sub_folder+imitated_SwarmEnvAreasWidth" +
                    to_string((i)) + ".csv");
            for (int i = 0; i < get<2>(population_Fit_Env[0]).areas_num; i++) {
                myfile14 << get<2>(population_Fit_Env[0]).areas_X[i] << ",";
                myfile15 << get<2>(population_Fit_Env[0]).areas_Y[i] << ",";
                myfile16 << get<2>(population_Fit_Env[0]).areas_Width[i] << ",";
            }
            myfile14.close();
            myfile15.close();
            myfile16.close();
        } else
            myfile13 << 0 << ",";
        if (get<2>(population_Fit_Env[0]).objects_in_env) {
            myfile13 << 1 << ",";

            std::ofstream myfile14;
            myfile14.open(
                    sub_folder+imitated_SwarmEnvObjectX" +
                    to_string((i)) + ".csv");
            std::ofstream myfile15;
            myfile15.open(
                    sub_folder+imitated_SwarmEnvObjectY" +
                    to_string((i)) + ".csv");
            for (int i = 0; i < blackboard::simulation_timesteps; i++) {
                for (int j = 0; j < get<2>(population_Fit_Env[0]).objects_num; j++) {
                    myfile14 << get<2>(population_Fit_Env[0]).objects_Data_X[i][j] << ",";
                    myfile15 << get<2>(population_Fit_Env[0]).objects_Data_Y[i][j] << ",";

                }
                myfile14 << "\n";
                myfile15 << "\n";

            }
            myfile14.close();
            myfile15.close();

        } else
            myfile13 << 0 << ",";
        if (get<2>(population_Fit_Env[0]).obstacles_in_env) {
            myfile13 << 1 << ",";
            std::ofstream myfile14;
            myfile14.open(
                    sub_folder+imitated_SwarmEnvObstaclesX" +
                    to_string((i)) + ".csv");
            std::ofstream myfile15;
            myfile15.open(
                    sub_folder+imitated_SwarmEnvObstaclesY" +
                    to_string((i)) + ".csv");
            std::ofstream myfile16;
            myfile16.open(
                    sub_folder+imitated_SwarmEnvObstaclesX2" +
                    to_string((i)) + ".csv");
            std::ofstream myfile17;
            myfile17.open(
                    sub_folder+imitated_SwarmEnvObstaclesY2" +
                    to_string((i)) + ".csv");
            for (int i = 0; i < get<2>(population_Fit_Env[0]).obstacles_num; i++) {
                myfile14 << get<2>(population_Fit_Env[0]).obstacles_X[i] << ",";
                myfile15 << get<2>(population_Fit_Env[0]).obstacles_Y[i] << ",";
                myfile16 << get<2>(population_Fit_Env[0]).obstacles_X2[i] << ",";
                myfile17 << get<2>(population_Fit_Env[0]).obstacles_Y2[i] << ",";

            }
            myfile14.close();
            myfile15.close();
            myfile16.close();
            myfile17.close();

        } else
            myfile13 << 0 << ",";
        myfile13.close();
        std::ofstream myfile18;
        myfile18.open(
                sub_folder+imitated_BT" + to_string((i)) +
                ".csv");
        for (int j = 0; j < BTree.size(); j++) {
            myfile18 << BTree[j].node << ",";

        }
        myfile18 << "\n";
        for (int j = 0; j < BTree.size(); j++) {
            myfile18 << BTree[j].child_num << ",";
        }
        myfile18.close();

        std::ofstream myfile118;
        myfile118.open(
                sub_folder+Best_BTs_" + to_string((i)) +
                ".csv");
        for (int b = 0; b < generations_number; b++) {
            for (int j = 0; j < all_BestBT[b].size(); j++) {
                myfile118 << all_BestBT[b][j].node;
                if (j != all_BestBT[b].size() - 1) {
                    myfile118 << ",";
                }

            }
            myfile118 << "\n";
            for (int j = 0; j < all_BestBT[b].size(); j++) {
                myfile118 << all_BestBT[b][j].child_num;
                if (j != all_BestBT[b].size() - 1) {
                    myfile118 << ",";
                }
            }
            myfile118 << "\n";
        }
        myfile118.close();

        std::ofstream myfile30;
        myfile30.open(
                sub_folder+Worst_BTs_" + to_string((i)) +
                ".csv");
        for (int b = 0; b < generations_number; b++) {
            for (int j = 0; j < all_worstBT[b].size(); j++) {
                myfile30 << all_worstBT[b][j].node;
                if (j != all_worstBT[b].size() - 1) {
                    myfile30 << ",";
                }

            }
            myfile30 << "\n";
            for (int j = 0; j < all_worstBT[b].size(); j++) {
                myfile30 << all_worstBT[b][j].child_num;
                if (j != all_worstBT[b].size() - 1) {
                    myfile30 << ",";
                }
            }
            myfile30 << "\n";
        }
        myfile30.close();

        std::ofstream myfile19;
        myfile19.open(
                sub_folder+BestFitness" + to_string((i)) +
                ".csv");

        for (int i = 0; i < Best_Fitness.size(); i++) {
            myfile19 << Best_Fitness[i] << ",";
        }
        myfile19 << "\n";
        myfile19.close();

        std::ofstream myfile20;
        myfile20.open(
                sub_folder+Avg_Fitness" + to_string((i)) +
                ".csv");

        for (int i = 0; i < Avg_Fitness.size(); i++) {
            myfile20 << Avg_Fitness[i] << ",";
        }
        myfile20 << "\n";
        myfile20.close();

        std::ofstream myfile40;
        myfile40.open(
                sub_folder+Max_Fitness" + to_string((i)) +
                ".csv");

        for (int i = 0; i < Max_Fitness.size(); i++) {
            myfile40 << Max_Fitness[i] << ",";
        }
        myfile40 << "\n";
        myfile40.close();

        std::ofstream myfile21;
        myfile21.open(sub_folder+BestBT_Metrics" +
                      std::to_string(i) + ".csv");
        for (int i = 0; i < blackboard::simulation_timesteps; i++) {
            for (int j = 0; j < BestBT_metrics.size(); j++) {
                myfile21 << BestBT_metrics[j][i] << ",";
            }
            myfile21 << "\n";
        }
        myfile21.close();

        std::ofstream myfile22;
        myfile22.open(sub_folder+OriginalBT_Metrics" +
                      std::to_string(i) + ".csv");

        for (int i = 0; i < blackboard::simulation_timesteps; i++) {
            for (int j = 0; j < original_metrics.size(); j++) {
                myfile22 << original_metrics[j][i] << ",";
            }
            myfile22 << "\n";
        }
        myfile22.close();

        std::ofstream myfile23;
        myfile23.open(
                sub_folder+All_BTs" + std::to_string(i) +
                ".csv");

        for (int i = 0; i < all_BT.size(); i++) {
            for (int j = 0; j < all_BT[i].size(); j++) {

                for (int k = 0; k < all_BT[i][j].size(); k++) {

                    myfile23 << all_BT[i][j][k].node << ",";
                }
                myfile23 << "\n";

                for (int k = 0; k < all_BT[i][j].size(); k++) {

                    myfile23 << all_BT[i][j][k].child_num << ",";
                }
                myfile23 << "\n";
            }
        }
        myfile23.close();



    }



};

#endif //SWARM_BTEVOLUTION_H

