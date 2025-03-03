//
// Created by kholood alharthi on 10/19/21.
//

#include "blackboard.h"
#include <cmath>
#include <random>


#ifndef SWARM_NODE_H
#define SWARM_NODE_H


string SUCCESS = "Success";
string FAILURE = "Failure";
string RUNNING = "Running";
constexpr double PI = 3.14159265358;
int node_idx;
//enum NodeState
//{
//    RUNNING,
//    SUCCESS,
//    FAILURE
//};

class Node {
public :

    blackboard b;

    virtual string tick() {
        return FAILURE;
    }
};

class SeqNode : public Node {
public:
    stack<Node *> childStack;
    stack<Node *> childStackTemp;
    stack<int> child_ind;
    stack<int> child_ind_temp;
    blackboard *blackboard;


    SeqNode(stack<Node *> sub_tree, stack<int> sub_tree_indx) {
        childStack = sub_tree;
        childStackTemp = sub_tree;
        child_ind = sub_tree_indx;
        child_ind_temp = sub_tree_indx;

    }

    string tick() {

        childStack = childStackTemp;
        child_ind = child_ind_temp;

        while (!childStack.empty()) {
            node_idx = child_ind.top();
            string result = childStack.top()->tick();
            childStack.pop();
            child_ind.pop();

            if (result == RUNNING) {
                return RUNNING;
            }
            if (result == FAILURE) {
                return FAILURE;
            }

        }


        return SUCCESS;
    }
};

class SelNode : public Node {
public:
    blackboard *blackboard;
    stack<Node *> childStack;
    stack<Node *> childStackTemp;
    stack<int> child_ind;
    stack<int> child_ind_temp;


    SelNode(stack<Node *> sub_tree, stack<int> sub_tree_indx) {
        childStack = sub_tree;
        childStackTemp = sub_tree;
        child_ind = sub_tree_indx;
        child_ind_temp = sub_tree_indx;
    }

    virtual string tick() {
        childStack = childStackTemp;
        child_ind = child_ind_temp;
        while (!childStack.empty()) {
            node_idx = child_ind.top();
            string result = childStack.top()->tick();
            childStack.pop();
            child_ind.pop();
            if (result == RUNNING) {
                return RUNNING;
            }
            if (result == SUCCESS) {
                return SUCCESS;
            }

        }
        return FAILURE;
    }

};


class Repulsion_Node : public Node {
public :
    blackboard *blackboard;

    Repulsion_Node(struct blackboard &temp) {
        blackboard = &temp;
    }

    string tick() {


            blackboard->repulsion_tick=1;

            double neighbors = 0;
            double center = 0.0;
            double delta_x = 0.0;
            double delta_y = 0.0;
            double center_x = 0.0;
            double center_y = 0.0;
            double random_power = 2;
            float decay=0.001;
            float repulsive_force=30;
            float heading=0;

            blackboard->forces += 1;


        //for (int j = 0; j < blackboard->BT_Env.swarm_size; j++) {
        for (int j = 0; j < blackboard::Swarm_size; j++) {
                if (blackboard->Distances[blackboard->agentID][j] < blackboard::local_radius + blackboard::robot_radius &&
                    j != blackboard->agentID) {
                    center_x = center_x + ( repulsive_force*exp(-blackboard->Distances[blackboard->agentID][j]*decay) *
                                            (blackboard->DistancesX[blackboard->agentID][j]/blackboard->Distances[blackboard->agentID][j]));
                    center_y = center_y + ( repulsive_force*exp(-blackboard->Distances[blackboard->agentID][j]*decay) *
                                            (blackboard->DistancesY[blackboard->agentID][j]/blackboard->Distances[blackboard->agentID][j]));

                    /*      delta_x = blackboard->DistancesX[blackboard->agentID][j] / blackboard->Distances[blackboard->agentID][j];
                          delta_y = blackboard->DistancesY[blackboard->agentID][j] / blackboard->Distances[blackboard->agentID][j];

                          center = 50 / (blackboard->DistancesY[blackboard->agentID][j]* blackboard->DistancesY[blackboard->agentID][j]);

                          center_x += center * delta_x;
                          center_y += center * delta_y;*/


                    //center_x = center_x + (blackboard->agent.x - blackboard->DistancesX[blackboard->agentID][j]) ;
                    //center_y = center_y + (blackboard->agent.y - blackboard->DistancesY[blackboard->agentID][j]);
                    neighbors += 1;
                }
            }

            if (neighbors > 0 and blackboard->repulsion_force!=1) {

                blackboard->x_neighbour_force += (center_x);
                blackboard->y_neighbour_force += (center_y);
                blackboard->repulsion_force=1;

                blackboard->Nodes_success_times[node_idx] += 1;
                heading =  atan2(center_y,center_x);



                //blackboard->x_force+=  (blackboard->agent.x - (center_x/neighbors));
                //blackboard->y_force+=  (blackboard->agent.y - (center_y/neighbors));

                //blackboard->x_force -= (center_x/neighbors  - blackboard->agent.x);
                //blackboard->y_force -= (center_y/neighbors  - blackboard->agent.y);


            }

            double min = -50, max = 50;


            blackboard->agent.heading += ((0.005 * (min + (rand() / (RAND_MAX / (max - min)))))+heading);

            blackboard->x_neighbour_force += cos(blackboard->agent.heading);
            blackboard->y_neighbour_force += sin(blackboard->agent.heading);





        blackboard->Nodes_tick_times[node_idx] += 1;

        return SUCCESS;
    }
};


class AggregationNode : public Node {
public :
    blackboard *blackboard;

    AggregationNode(struct blackboard &temp) {
        blackboard = &temp;

    }

    string tick() {


            blackboard->attraction_tick = 1;
            double neighbors = 0.0;
            double center_x = 0.0;
            double center_y = 0.0;
            float decay=0.01;
            float attraction_force=5;

            blackboard->forces += 1;


/*        for (int j = 0; j < blackboard::Swarm_size; j++) {
            if (blackboard->Distances[blackboard->agentID][j] < blackboard::local_radius + blackboard::robot_radius &&
                j != blackboard->agentID) {
                center_x = center_x + (blackboard->agent.x - blackboard->DistancesX[blackboard->agentID][j]);
                center_y = center_y + (blackboard->agent.y - blackboard->DistancesY[blackboard->agentID][j]);
                neighbors += 1;
            }
        }
        if (neighbors > 0) {
            blackboard->x_force += 0.1 * (center_x / neighbors - blackboard->agent.x);
            blackboard->y_force += 0.1 * (center_y / neighbors - blackboard->agent.y);
            blackboard->agent.motion_force = 3;
            blackboard->separation_dist = 10;
            blackboard->separation_force = 8;
            blackboard->Nodes_success_times[node_idx] += 1;
        }*/


        //for (int j = 0; j < blackboard->BT_Env.swarm_size; j++) {
            for (int j = 0; j < blackboard::Swarm_size; j++) {
                if (blackboard->Distances[blackboard->agentID][j] < blackboard::local_radius + blackboard::robot_radius &&
                    j != blackboard->agentID) {
                    center_x = center_x + ( attraction_force*exp(-blackboard->Distances[blackboard->agentID][j]*decay) *
                                            (blackboard->DistancesX[blackboard->agentID][j]/blackboard->Distances[blackboard->agentID][j]));
                    center_y = center_y + ( attraction_force*exp(-blackboard->Distances[blackboard->agentID][j]*decay) *
                                            (blackboard->DistancesY[blackboard->agentID][j]/blackboard->Distances[blackboard->agentID][j]));

                    neighbors += 1;
                }
            }
            if (neighbors > 0 and blackboard->attraction_force!=1) {
                blackboard->x_neighbour_force += (-center_x);
                blackboard->y_neighbour_force += (-center_y);
                blackboard->attraction_force = 1;

                blackboard->Nodes_success_times[node_idx] += 1;
            }

            double min = -50, max = 50;
            blackboard->agent.heading += 0.01 * (min + (rand() % static_cast<int>(max - min + 1)));

            blackboard->x_neighbour_force += cos(blackboard->agent.heading);
            blackboard->y_neighbour_force += sin(blackboard->agent.heading);


        blackboard->Nodes_tick_times[node_idx] += 1;
        return SUCCESS;

    }
};


class SE_Node : public Node {
public :
    blackboard *blackboard;

    SE_Node(struct blackboard &temp) {
        blackboard = &temp;
    }

    string tick() {

        if (blackboard->SE_force!=1)
        {
            blackboard->SE_force=1;
            blackboard->forces += 1;
            blackboard->NEWS_forces += 1;

            double min = (3 * PI) / 2, max = 2 * PI;
            double r = (double) rand() / RAND_MAX;
            double heading(min + r * (max - min));

            blackboard->x_NEWS += cos(heading);
            blackboard->y_NEWS += sin(heading);
        }


        blackboard->Nodes_tick_times[node_idx] += 1;
        blackboard->Nodes_success_times[node_idx] += 1;
        return SUCCESS;
    }
};

class NW_Node : public Node {
public :
    blackboard *blackboard;

    NW_Node(struct blackboard &temp) {
        blackboard = &temp;
    }

    string tick() {

        if(blackboard->NW_force!=1)
        {
            blackboard->NW_force=1;
            blackboard->forces += 1;
            blackboard->NEWS_forces += 1;


            double min = PI / 2, max = PI;

            double r = (double) rand() / RAND_MAX;
            double heading = min + r * (max - min);
            //blackboard->agent.heading+= 0.01*heading;

            blackboard->x_NEWS += cos(heading);
            blackboard->y_NEWS += sin(heading);
        }



        blackboard->Nodes_tick_times[node_idx] += 1;
        blackboard->Nodes_success_times[node_idx] += 1;
        return SUCCESS;
    }
};


class SW_Node : public Node {
public :
    blackboard *blackboard;

    SW_Node(struct blackboard &temp) {
        blackboard = &temp;
    }

    string tick() {
        if (blackboard->SW_force!=1)
        {
            blackboard->SW_force=1;
            blackboard->forces += 1;
            blackboard->NEWS_forces += 1;

            double min = PI, max = (3 * PI) / 2;
            double r = (double) rand() / RAND_MAX;
            double heading = min + r * (max - min);
            blackboard->x_NEWS += cos(heading);
            blackboard->y_NEWS += sin(heading);
        }

        blackboard->Nodes_tick_times[node_idx] += 1;
        blackboard->Nodes_success_times[node_idx] += 1;
        return SUCCESS;
    }
};


//Action Node S-W Force

class NE_Node : public Node {

public :

    blackboard *blackboard;

    NE_Node(struct blackboard &temp) {
        blackboard = &temp;

    }

    string tick() {
        if (blackboard->NE_force!=1) {
            blackboard->NE_force = 1;
            blackboard->forces += 1;
            blackboard->NEWS_forces += 1;


            double min = 0, max = PI / 2;

            double r = (double) rand() / RAND_MAX;
            double heading = min + r * (max - min);
            blackboard->x_NEWS += cos(heading);
            blackboard->y_NEWS += sin(heading);
        }





        blackboard->Nodes_tick_times[node_idx] += 1;
        blackboard->Nodes_success_times[node_idx] += 1;
        return SUCCESS;
    }

};

//Action Node Random Motion

class RandNode : public Node {

public :
    blackboard *blackboard;


    RandNode(struct blackboard &temp) {
        blackboard = &temp;

    }


    string tick() {

        blackboard->forces += 1;


        int min = -50, max = 50;
        blackboard->agent.heading += 0.01 * (min + (rand() % static_cast<int>(max - min + 1)));
        blackboard->x_force += cos(blackboard->agent.heading);
        blackboard->y_force += sin(blackboard->agent.heading);



        //blackboard->x_force+= blackboard->agent.x+(min + (rand() / (RAND_MAX / (max - min))));
        //blackboard->y_force+= blackboard->agent.y+(min + (rand() / (RAND_MAX / (max - min))));



        blackboard->Nodes_tick_times[node_idx] += 1;
        blackboard->Nodes_success_times[node_idx] += 1;
        return SUCCESS;
    }

};

//Action Node Random Motion

class Boundary_Turn : public Node {

public :
    blackboard *blackboard;


    Boundary_Turn(struct blackboard &temp) {
        blackboard = &temp;

    }


    string tick() {



        double margin = 25;
        double Velx = 0.0;
        double Vely = 0.0;
        double heading = 0.0;
        double delta_x = 0.0;
        double delta_y = 0.0;
        double boundary_force = 60;


        if (blackboard->agent.x > 250 - margin) {

            Velx = -1;
            Vely = 1;
            heading = atan2(Vely, Velx);
            //delta_x = boundary_force * blackboard::Swarm_speed * cos(heading) * blackboard::Swarm_time_step;
            blackboard->x_force -= boundary_force ;
            blackboard->separation_dist += 10;
            blackboard->separation_force += 5;
            blackboard->separation_decay = 0.001;
            blackboard->boundary_force = 1;


        }
        if (blackboard->agent.x < -250 + margin) {
            Velx = 1;
            Vely = 1;
            heading = atan2(Vely, Velx);
            blackboard->x_force += boundary_force;
            blackboard->separation_dist += 10;
            blackboard->separation_force += 5;
            blackboard->separation_decay = 0.001;
            blackboard->boundary_force = 1;
            //delta_x = boundary_force * blackboard::Swarm_speed * cos(heading) * blackboard::Swarm_time_step;
        }
        if (blackboard->agent.y > 250 - margin) {
            Velx = 1;
            Vely = -1;
            heading = atan2(Vely, Velx);
            blackboard->y_force -= boundary_force ;
            blackboard->separation_dist += 10;
            blackboard->separation_force += 5;
            blackboard->separation_decay = 0.001;
            blackboard->boundary_force = 1;
            //delta_y = boundary_force * blackboard::Swarm_speed * sin(heading) * blackboard::Swarm_time_step;
        }
        if (blackboard->agent.y < -250 + margin) {
            Velx = 1;
            Vely = 1;
            heading = atan2(Vely, Velx);
            blackboard->y_force += boundary_force ;
            blackboard->separation_dist += 10;
            blackboard->separation_force += 5;
            blackboard->separation_decay = 0.001;
            blackboard->boundary_force = 1;
            //delta_y = boundary_force * blackboard::Swarm_speed * sin(heading) * blackboard::Swarm_time_step;
        }

        if(blackboard->agent.x > 250 - margin or blackboard->agent.x < -250 + margin or
                blackboard->agent.y > 250 - margin or blackboard->agent.y < -250 + margin)
        {
            blackboard->Nodes_success_times[node_idx] += 1;

        }


        blackboard->Nodes_tick_times[node_idx] += 1;
        return SUCCESS;
    }

};



class Out_Boundary : public Node {

public :
    blackboard *blackboard;


    Out_Boundary(struct blackboard &temp) {
        blackboard = &temp;

    }


    string tick() {
        blackboard->Nodes_tick_times[node_idx] += 1;

        double margin = 3;

        if (abs(blackboard->agent.x) > 250 - margin
            || abs(blackboard->agent.y) > 250 - margin) {
            blackboard->Nodes_success_times[node_idx] += 1;
            return SUCCESS;

        } else {
            return FAILURE;

        }

    }

};




class High_density : public Node {

public :
    blackboard *blackboard;

    High_density(struct blackboard &temp) {
        blackboard = &temp;
    }

    string tick() {
        blackboard->Nodes_tick_times[node_idx] += 1;
        int local_density = 0;
        for (int j = 0; j < blackboard::Swarm_size; j++) {
            if (blackboard->Distances[blackboard->agentID][j] <
                blackboard::local_radius + blackboard::robot_radius && j != blackboard->agentID) {
                local_density += 1;
            }
        }

        if (local_density >= blackboard::High_density) {
            blackboard->Nodes_success_times[node_idx] += 1;
            return SUCCESS;
        } else
            return FAILURE;
    }
};

class Low_density : public Node {

public :
    blackboard *blackboard;

    Low_density(struct blackboard &temp) {
        blackboard = &temp;
    }

    string tick() {
        blackboard->Nodes_tick_times[node_idx] += 1;

        int local_density = 0;

        //for (int j = 0; j < blackboard->BT_Env.swarm_size; j++) {
        for (int j = 0; j < blackboard::Swarm_size; j++) {
            if (blackboard->Distances[blackboard->agentID][j] <=
                        (blackboard::neighbours_in_radius) && j != blackboard->agentID) {
               // cout<<" agent "<< blackboard->agentID << " distance to" << j <<" is "<<blackboard->Distances[blackboard->agentID][j]<<endl;
                local_density += 1;
            }
        }


        if (local_density == 0) {

            blackboard->Nodes_success_times[node_idx] += 1;
            return SUCCESS;
        } else
        {
            return FAILURE;
        }
    }
};

class wait : public Node {

public :
    blackboard *blackboard;

    wait(struct blackboard &temp) {
        blackboard = &temp;
    }

    string tick() {
        blackboard->Nodes_tick_times[node_idx] += 1;



        if (blackboard->tick_times[blackboard->agentID] > blackboard::wait_time) {
            blackboard->Nodes_success_times[node_idx] += 1;
            return SUCCESS;
        } else
            return FAILURE;
    }
};

class Inside_Area : public Node {

public :
    blackboard *blackboard;

    Inside_Area(struct blackboard &temp) {
        blackboard = &temp;
    }

    string tick() {
        blackboard->Nodes_tick_times[node_idx] += 1;
        if (blackboard->BT_Env.areas_in_env) {

            int areas_in=0;

            for (int i = 0; i < blackboard->BT_Env.areas_num; i++) {
                if ((blackboard->agent.x-blackboard->robot_radius) > (blackboard->BT_Env.areas_X[i]) &&
                    (blackboard->agent.x+blackboard->robot_radius) <
                    (blackboard->BT_Env.areas_X[i] + blackboard->BT_Env.areas_Width[i]) &&
                    (blackboard->agent.y-blackboard->robot_radius) > (blackboard->BT_Env.areas_Y[i]) &&
                    (blackboard->agent.y+blackboard->robot_radius) <
                    (blackboard->BT_Env.areas_Y[i] + blackboard->BT_Env.areas_Width[i])) {
                    blackboard->Env_metrics.inside_area_times += 1;
                    blackboard->Nodes_success_times[node_idx] += 1;


                    areas_in+=1;
                }
            }

            if (areas_in>0)
            {
                return SUCCESS;
            }
            else
            {
                return FAILURE;
            }




        }
    }

};

class Area_inside_Radius : public Node {

public :
    blackboard *blackboard;

    Area_inside_Radius(struct blackboard &temp) {
        blackboard = &temp;
    }

    string tick() {
        blackboard->Nodes_tick_times[node_idx] += 1;
        if (blackboard->BT_Env.areas_in_env) {

            int area_sensed =0;

            for (int i = 0; i < blackboard->BT_Env.areas_num; i++) {
                double area_center_X = blackboard->BT_Env.areas_X[i] + (blackboard->BT_Env.areas_Width[i] / 2);
                double area_center_Y = blackboard->BT_Env.areas_Y[i] + (blackboard->BT_Env.areas_Width[i]  /2);

                //assuming the area (Squares) Circumscribed by Circle
                // radius of circle is sqrt(2) times square width divided by 2

                //double area_radius = (sqrt(2) * blackboard->BT_Env.areas_Width[i]) / 2;
                //double dist1 = sqrt(
                       // pow(blackboard->agent.x - area_center_X, 2) + pow(blackboard->agent.y - area_center_Y, 2));

                /*float bottomLeft_x =blackboard->BT_Env.areas_X[i] +(blackboard->BT_Env.areas_Width[i] / 5);
                float bottomLeft_y= blackboard->BT_Env.areas_Y[i] +(blackboard->BT_Env.areas_Width[i] / 5);
                float topLeft_x = bottomLeft_x;
                float topLeft_y =  bottomLeft_y +(blackboard->BT_Env.areas_Width[i] / 2);
                float topRight_x = bottomLeft_x + (blackboard->BT_Env.areas_Width[i] / 2);
                float topRight_y = bottomLeft_y + (blackboard->BT_Env.areas_Width[i] / 2);
                float bottomRight_x = bottomLeft_x + (blackboard->BT_Env.areas_Width[i] / 2);
                float bottomRight_y =bottomLeft_y;*/

         /*       float bottomLeft_x =area_center_X -(blackboard->BT_Env.areas_Width[i] / 2);
                float bottomLeft_y= area_center_Y -(blackboard->BT_Env.areas_Width[i] / 2);
                float topLeft_x = area_center_X -(blackboard->BT_Env.areas_Width[i] / 2);
                float topLeft_y =  area_center_Y +(blackboard->BT_Env.areas_Width[i] / 2);
                float topRight_x = area_center_X +(blackboard->BT_Env.areas_Width[i] / 2);
                float topRight_y = area_center_Y +(blackboard->BT_Env.areas_Width[i] / 2);
                float bottomRight_x = area_center_X +(blackboard->BT_Env.areas_Width[i] / 2);
                float bottomRight_y =area_center_Y -(blackboard->BT_Env.areas_Width[i] / 2);

*/

                // Calculate the distance from the circle center to each corner of the rectangle
//                double distanceTopLeft = std::sqrt(std::pow(blackboard->agent.x  - topLeft_x, 2) + std::pow(blackboard->agent.y  - topLeft_y, 2));
//                double distanceTopRight = std::sqrt(std::pow(blackboard->agent.x  - topRight_x, 2) + std::pow(blackboard->agent.y  - topRight_y, 2));
//                double distanceBottomLeft = std::sqrt(std::pow(blackboard->agent.x  - bottomLeft_x, 2) + std::pow(blackboard->agent.y  - bottomLeft_y, 2));
//                double distanceBottomRight = std::sqrt(std::pow(blackboard->agent.x  - bottomRight_x, 2) + std::pow(blackboard->agent.y  - bottomRight_y, 2));
//                if ((distanceTopLeft <= blackboard::local_radius) or
//                    (distanceTopRight <= blackboard::local_radius) or
//                    (distanceBottomLeft <= blackboard::local_radius) or
//                    (distanceBottomRight <= blackboard::local_radius) or
//                    if((abs(blackboard->agent.x - area_center_X) <= (blackboard->BT_Env.areas_Width[i] / 2) + blackboard::local_radius and
//                        abs(blackboard->agent.y - area_center_Y) <= (blackboard->BT_Env.areas_Width[i] / 2) + blackboard::local_radius)){
//                    area_sensed+=1;
//                }


                float closest_x = max(min(blackboard->agent.x, area_center_X + blackboard->BT_Env.areas_Width[i]/2), area_center_X - blackboard->BT_Env.areas_Width[i]/2);
                float closest_y = max(min(blackboard->agent.y , area_center_Y + blackboard->BT_Env.areas_Width[i]/2), area_center_Y - blackboard->BT_Env.areas_Width[i]/2);
                double dist1 = sqrt(
                        pow(blackboard->agent.x - closest_x, 2) + pow(blackboard->agent.y - closest_y, 2));
                 //dist1 <= (blackboard::local_radius +(blackboard->BT_Env.areas_Width[i] / 2))

                if (dist1 <= (blackboard::local_radius )) {
                    area_sensed+=1;
                }
            }

            if (area_sensed>0)
            {
                blackboard->Nodes_success_times[node_idx] += 1;
                return SUCCESS;

            }
            else
            {
                return FAILURE;
            }




        }
    }

};


class Pick_Object : public Node {
public :
    blackboard *blackboard;

    Pick_Object(struct blackboard &temp) {
        blackboard = &temp;
    }

    string tick() {
        blackboard->Nodes_tick_times[node_idx] += 1;

        if (blackboard->BT_Env.objects_in_env) {
            if (blackboard->agent.picked_object) {
                blackboard->Nodes_success_times[node_idx] += 1;
                return SUCCESS;
            }

            if (!blackboard->agent.picked_object) {
                for (int j = 0; j < blackboard->BT_Env.objects_num; j++) {
                    double dist1 = sqrt(pow(blackboard->agent.x - (blackboard->BT_Env.objects_X[j]), 2) +
                                        pow(blackboard->agent.y - (blackboard->BT_Env.objects_Y[j]), 2));

                    if (dist1 <= (blackboard::robot_radius+(blackboard->BT_Env.objects_Width2))) {
                        if (!blackboard->BT_Env.objects_picked[j]) {
                            blackboard->BT_Env.objects_picked[j] = true;
                            blackboard->agent.picked_object = true;
                            blackboard->agent.object_ID = j;
                            blackboard->Env_metrics.picked_obj_num += 1;
                            blackboard->BT_Env.objects_X[j] = (blackboard->agent.x);
                            blackboard->BT_Env.objects_Y[j] = (blackboard->agent.y);
                            blackboard->Nodes_success_times[node_idx] += 1;


                        }
                    }
                }
            }


        }

        return SUCCESS;

    }
};


class Drop_Object : public Node {
public :

    blackboard *blackboard;

    Drop_Object(struct blackboard &temp) {
        blackboard = &temp;

    }


    string tick() {
        blackboard->Nodes_tick_times[node_idx] += 1;

        if (blackboard->BT_Env.objects_in_env) {
            if (blackboard->agent.picked_object) {
                blackboard->agent.picked_object = false;
                blackboard->agent.picked_object_t=0;
                blackboard->agent.object_ID = -1;
                blackboard->Env_metrics.drop_obj_num += 1;
                blackboard->Nodes_success_times[node_idx] += 1;

            }
        }

        return SUCCESS;
    }

};


class If_picked_object : public Node {
public :

    blackboard *blackboard;

    If_picked_object(struct blackboard &temp) {
        blackboard = &temp;

    }

    string tick() {
        blackboard->Nodes_tick_times[node_idx] += 1;

        if (blackboard->agent.picked_object) {
            blackboard->Nodes_success_times[node_idx] += 1;
            return SUCCESS;
        }
        else
        {
            return FAILURE;
        }
    }

};

class If_neighbour_picked_object : public Node {
public :

    blackboard *blackboard;

    If_neighbour_picked_object(struct blackboard &temp) {
        blackboard = &temp;

    }

    string tick() {
        blackboard->Nodes_tick_times[node_idx] += 1;

        int neighbour_picked_object=0;
        for (int j = 0; j < blackboard::Swarm_size; j++) {
            if (blackboard->Distances[blackboard->agentID][j] <
                (blackboard::local_radius + blackboard::robot_radius) && j != blackboard->agentID && blackboard->agents[j].picked_object==true) {
                neighbour_picked_object+=1;

            }
        }
        if (neighbour_picked_object>0) {
            blackboard->Nodes_success_times[node_idx] += 1;
            return SUCCESS;
        }
        else
        {
            return FAILURE;
        }
    }

};

class If_picked_object_t : public Node {
public :

    blackboard *blackboard;

    If_picked_object_t(struct blackboard &temp) {
        blackboard = &temp;

    }

    string tick() {
        blackboard->Nodes_tick_times[node_idx] += 1;

        if (blackboard->agent.picked_object_t>=100) {
            blackboard->Nodes_success_times[node_idx] += 1;
            return SUCCESS;
        }
        else
        {
            return FAILURE;
        }
    }

};

class If_inside_area_t : public Node {
public :

    blackboard *blackboard;

    If_inside_area_t(struct blackboard &temp) {
        blackboard = &temp;

    }

    string tick() {
        blackboard->Nodes_tick_times[node_idx] += 1;

        if (blackboard->agent.inside_area_t>=50) {
            blackboard->Nodes_success_times[node_idx] += 1;
            return SUCCESS;
        }
        else
        {
            return FAILURE;
        }
    }

};

class If_neighbour_inside_area : public Node {
public :

    blackboard *blackboard;

    If_neighbour_inside_area(struct blackboard &temp) {
        blackboard = &temp;

    }

    string tick() {
        blackboard->Nodes_tick_times[node_idx] += 1;

        int neighbour_inside_area=0;
        for (int j = 0; j < blackboard::Swarm_size; j++) {
            if (blackboard->Distances[blackboard->agentID][j] <
                (blackboard::local_radius + blackboard::robot_radius) && j != blackboard->agentID && blackboard->agents[j].inside_area==true) {
                neighbour_inside_area+=1;

            }
        }
        if (neighbour_inside_area>0) {
            blackboard->Nodes_success_times[node_idx] += 1;
            return SUCCESS;
        }
        else
        {
            return FAILURE;
        }
    }

};


class Success : public Node {
public :
    blackboard *blackboard;

    Success(struct blackboard &temp) {
        blackboard = &temp;
    }

    string tick() {
        return SUCCESS;
    }

};

class failure : public Node {
public :
    blackboard *blackboard;

    failure(struct blackboard &temp) {
        blackboard = &temp;
    }

    string tick() {
        return FAILURE;
    }

};



#endif //SWARM_NODE_H
