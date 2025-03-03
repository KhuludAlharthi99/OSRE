

#ifndef MAIN_CPP_BEHAVIORTREE_H
#define MAIN_CPP_BEHAVIORTREE_H

#include "Node.h"
#include "blackboard.h"



class BehaviorTree {

public:
    stack<Node *> sub_tree;
    stack<Node *> sub_tree2;

    stack<int> indx_sub_tree;
    stack<int> indx_sub_tree2;

    Node *root;
    blackboard blackboard;
    vector<blackboard::Node> BT;
    int node_index = 0;
    int local_index = 0;
    int final_child=5 ;
    int subtree_size;



    blackboard::env BT_environment;

    blackboard::Environment Env_settings;

    vector<int> Leaf_nodes;
    vector<int> Leaf_nodes_0;

    vector<int> non_conditions_nodes;
    vector<int> conditions_nodes;


    vector<int> BT_nodes;
    int send_msg_node=0;
    int read_msg_node=0;
    int sub_root;
    int last_added_node =0;
    int drop_object_node=0;


    int max = 250, min = -250;

    int generateRandomNumber(int min_val, int max_val) {
        // Seed the random number generator
        static std::random_device rd;
        static std::mt19937 gen(rd());

        // Define the distribution
        std::uniform_int_distribution<int> dis(min_val, max_val);

        // Generate and return the random number
        return dis(gen);
    }

    void Add_leaf_node2 ()
    {
        int random_index = (rand() % Leaf_nodes.size());
        BT.push_back({Leaf_nodes[random_index], 0});

        if(Leaf_nodes[random_index]==19 or
           Leaf_nodes[random_index]==14 or
           Leaf_nodes[random_index]==23 or
           Leaf_nodes[random_index]==24)
        {
            Leaf_nodes.push_back(18);
        }

        if(Leaf_nodes[random_index]==22)
        {
            Leaf_nodes.push_back(21);
        }
    }

    void Add_leaf_node2 (int node)
    {
        BT.push_back({node, 0});

        if(node==19 or
           node==14 or
           node==23 or
           node==24)
        {
            Leaf_nodes.push_back(18);
        }

        if(node==22)
        {
            Leaf_nodes.push_back(21);
        }
    }

    void Add_leaf_node ()
    {
        if (sub_root ==0) //Seq
        {
            if (final_child ==0)
            {
                int random_index = (rand() % non_conditions_nodes.size());
                BT.push_back({non_conditions_nodes[random_index], 0});
                last_added_node=non_conditions_nodes[random_index];


                if(non_conditions_nodes[random_index]==19 or
                   non_conditions_nodes[random_index]==14 or
                   non_conditions_nodes[random_index]==23 or
                   non_conditions_nodes[random_index]==24)
                {
                    non_conditions_nodes.push_back(18);
                    Leaf_nodes.push_back(18);
                    drop_object_node=1;

                }
                if(non_conditions_nodes[random_index]==21)
                {
                    send_msg_node=1;

                }
            }
            else
            {
                if (send_msg_node==1 && read_msg_node==0 && last_added_node!=21)
                {
                    Leaf_nodes.push_back(22);
                    conditions_nodes.push_back(22);
                }
                if (drop_object_node ==1)
                    {
                        int random_index = (rand() % conditions_nodes.size());
                        BT.push_back({conditions_nodes[random_index], 0});
                        last_added_node=conditions_nodes[random_index];

                        if(conditions_nodes[random_index]==22)
                        {
                            read_msg_node=1;

                        }
                        drop_object_node=0;
                    }
                    else
                    {
                        int random_index = (rand() % Leaf_nodes.size());
                        BT.push_back({Leaf_nodes[random_index], 0});
                        last_added_node=Leaf_nodes[random_index];


                        if(Leaf_nodes[random_index]==19 or
                           Leaf_nodes[random_index]==14 or
                           Leaf_nodes[random_index]==23 or
                           Leaf_nodes[random_index]==24 )
                        {
                            drop_object_node=1;
                            non_conditions_nodes.push_back(18);
                            Leaf_nodes.push_back(18);

                        }
                        if(Leaf_nodes[random_index]==21)
                        {
                            send_msg_node=1;

                        }
                        if(Leaf_nodes[random_index]==22)
                        {
                            read_msg_node=1;

                        }
                    }



            }

        }
        else if (sub_root ==1) //Sel
        {
            if (final_child==0)
            {
                int random_index = (rand() % non_conditions_nodes.size());
                BT.push_back({non_conditions_nodes[random_index], 0});


                 if(non_conditions_nodes[random_index]==19 or
                    non_conditions_nodes[random_index]==14 or
                    non_conditions_nodes[random_index]==23 or
                    non_conditions_nodes[random_index]==24)
                {
                    non_conditions_nodes.push_back(18);
                    Leaf_nodes.push_back(18);

                }
                if(non_conditions_nodes[random_index]==21)
                {
                    send_msg_node=1;

                }

            }
            else if (final_child ==1 || final_child ==2)
            {

                if (send_msg_node==1 && read_msg_node==0 && last_added_node!=21)
                {
                    Leaf_nodes.push_back(22);
                    conditions_nodes.push_back(22);
                }

                    int random_index = (rand() % conditions_nodes.size());
                    BT.push_back({conditions_nodes[random_index], 0});
                   last_added_node=conditions_nodes[random_index];

                    if(conditions_nodes[random_index]==22)
                    {
                        read_msg_node=1;

                    }



            }

        }
    }

    void Full(int depth, int max_depth, int breadth) {
        if (depth == max_depth) {
            Add_leaf_node();
            node_index += 1;
            return;

        } else {
            breadth = (rand() % 2) + 2;
            subtree_size=breadth;
            sub_root=(rand() % 2);
            BT.push_back({sub_root, breadth});
            node_index += 1;
            for (int i = 0; i < breadth; i++) {
                final_child = i;
                Full(depth + 1, max_depth, breadth);
            }
            return;

        }
    }

    void Grow(int depth, int max_depth, int breadth) {
        int random_index;
        if (depth == max_depth) {
            Add_leaf_node();
            node_index += 1;
            return;

        } else {
            if (node_index == 0) {
                breadth = (rand() % 2) + 2;
                subtree_size=breadth;
                sub_root=(rand() % 2);
                BT.push_back({sub_root, breadth});
                node_index += 1;
                for (int i = 0; i < breadth; i++) {
                    final_child=i;
                    Grow(depth + 1, max_depth, breadth);
                }
                return;
            } else {
                random_index = (rand() % BT_nodes.size());
                int node = BT_nodes[random_index];
                if (node == 0 || node == 1) {
                    sub_root = node;
                    breadth = (rand() % 2) + 2;
                    subtree_size=breadth;
                    BT.push_back({node, breadth});
                    node_index += 1;
                    for (int i = 0; i < breadth; i++) {
                        final_child=i;
                        Grow(depth + 1, max_depth, breadth);
                    }
                    return;
                } else {
                    Add_leaf_node();
                    node_index += 1;
                    return;
                }

            }
        }}





    void nActions_RandomBT_generator(int depth) {
        Leaf_nodes = Leaf_nodes_0;
        double random;
        node_index = 0;
        random = ((double) rand() / (RAND_MAX));
        int Tree_Depth = (rand() % depth) + 1;

        if (random < 0.5) // full method
        {

            Full(0, Tree_Depth, 0);
        } else // grow method
        {

            Grow(0, Tree_Depth, 0);

        }
    }





    void BT_nodes_set_env ()
    {
        for (int i =2 ; i<11; i++)
        {
            Leaf_nodes.push_back(i);
        }
        if (Env_settings.areas)
        {
            Leaf_nodes.push_back(11);
            Leaf_nodes.push_back(17);
            Leaf_nodes.push_back(18);

        }
        if (Env_settings.objects)
        {
            Leaf_nodes.push_back(12);
            Leaf_nodes.push_back(13);

        }
        if (Env_settings.obstacles)
        {
            Leaf_nodes.push_back(14);

        }
        if (Env_settings.msgs)
        {
            Leaf_nodes.push_back(15);
            Leaf_nodes.push_back(16);

        }
        BT_nodes = Leaf_nodes;
        BT_nodes.push_back(0);
        BT_nodes.push_back(1);
    }

    void BT_nodes_set_from_list (vector<int> actions)
    {
        Leaf_nodes = actions;
        Leaf_nodes_0= actions;
        BT_nodes = Leaf_nodes;
        BT_nodes.push_back(0);
        BT_nodes.push_back(1);
        for(int i=0;i<actions.size();i++)
        {
            if (actions[i] != 10 && actions[i] != 11
            && actions[i] != 12 && actions[i] != 13
            && actions[i] != 14 && actions[i] != 15
            && actions[i] != 16 && actions[i] != 17
            && actions[i] != 22 && actions[i] != 23 && actions[i] != 24)
            {
                non_conditions_nodes.push_back(actions[i]);
            }
            else
            {
                conditions_nodes.push_back(actions[i]);
            }
        }
    }



    void decode_BT4(int index) {
        if (index == -1) {
            return;
        }
        if (BT[index].node == 0) {
            for (int i = 0; i < BT[index].child_num; i++) {
                sub_tree2.push(sub_tree.top());
                sub_tree.pop();

                indx_sub_tree2.push(indx_sub_tree.top());
                indx_sub_tree.pop();
            }
            Node *node0 = new SeqNode(sub_tree2,indx_sub_tree2);

            for (int i = 0; i < BT[index].child_num; i++) {
                sub_tree2.pop();

                indx_sub_tree2.pop();
            }
            sub_tree.push(node0);

            indx_sub_tree.push(index);

            if (index == 0) {
                root = node0;
                return;
            }
            local_index -= 1;
            decode_BT4(local_index);
        } else if (BT[index].node == 1) {

            for (int i = 0; i < BT[index].child_num; i++) {
                sub_tree2.push(sub_tree.top());
                sub_tree.pop();

                indx_sub_tree2.push(indx_sub_tree.top());
                indx_sub_tree.pop();
            }
            Node *node1 = new SelNode(sub_tree2,indx_sub_tree2);
            for (int i = 0; i < BT[index].child_num; i++) {
                sub_tree2.pop();

                indx_sub_tree2.pop();

            }

            sub_tree.push(node1);

            indx_sub_tree.push(index);

            if (index == 0) {
                root = node1;
                return;
            }
            local_index -= 1;
            decode_BT4(local_index);
        }
         else {
            switch (BT[index].node) {
                case 2: {
                    Node *node2 = new Repulsion_Node(blackboard);
                    sub_tree.push(node2);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }

                case 3: {
                    Node *node3 = new AggregationNode(blackboard);
                    sub_tree.push(node3);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }

                case 4: {
                    Node *node4 = new NW_Node(blackboard);
                    sub_tree.push(node4);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }

                case 5: {
                    Node *node5 = new NE_Node(blackboard);
                    sub_tree.push(node5);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }

                case 6: {
                    Node *node6 = new SE_Node(blackboard);
                    sub_tree.push(node6);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }

                case 7: {
                    Node *node7 = new SW_Node(blackboard);
                    sub_tree.push(node7);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;

                }

                case 8: {
                    Node *node8= new RandNode(blackboard);
                    sub_tree.push(node8);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;

                }

                case 9: {
                    Node *node9 = new Boundary_Turn(blackboard);
                    sub_tree.push(node9);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }

                case 10: {
                    Node *node10 = new If_neighbour_inside_area(blackboard);
                    sub_tree.push(node10);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }

                case 11: {
                    Node *node11 = new If_inside_area_t(blackboard);
                    sub_tree.push(node11);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }

                case 12: {
                    Node *node12 = new wait(blackboard);
                    sub_tree.push(node12);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }

                case 13: {
                    Node *node13 = new Low_density(blackboard);
                    sub_tree.push(node13);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;

                }

                case 14: {
                    Node *node14 = new If_neighbour_picked_object(blackboard);
                    sub_tree.push(node14);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;

                }

                case 15: {
                    Node *node15 = new Out_Boundary(blackboard);
                    sub_tree.push(node15);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;

                }

                case 16: {
                    Node *node16 = new Inside_Area(blackboard);
                    sub_tree.push(node16);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;

                }

                case 17: {
                    Node *node17 = new Area_inside_Radius(blackboard);
                    sub_tree.push(node17);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;

                }

                case 18: {
                    Node *node18 = new Pick_Object(blackboard);
                    sub_tree.push(node18);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }
                case 19: {
                    Node *node19 = new Drop_Object(blackboard);
                    sub_tree.push(node19);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }

                case 20: {
                    Node *node20 = new Obstacles_Avoidance(blackboard);
                    sub_tree.push(node20);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }

                case 21: {
                    Node *node21 = new Send_Msg(blackboard);
                    sub_tree.push(node21);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }

                case 22: {
                    Node *node22 = new Received_Msg(blackboard);
                    sub_tree.push(node22);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }

                case 23: {
                    Node *node23 = new If_picked_object(blackboard);
                    sub_tree.push(node23);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }

                case 24: {
                    Node *node24 = new If_picked_object_t(blackboard);
                    sub_tree.push(node24);
                    indx_sub_tree.push(index);
                    local_index -= 1;
                    decode_BT4(local_index);
                    break;
                }
            }

        }
    }


    void reset_env ()
    {
        BT_environment.areas_in_env=false;
        BT_environment.objects_in_env=false;
        BT_environment.obstacles_in_env=false;

        BT_environment.areas_X.clear();
        BT_environment.areas_Y.clear();
        BT_environment.areas_Width.clear();

        BT_environment.obstacles_X.clear();
        BT_environment.obstacles_Y.clear();
        BT_environment.obstacles_X2.clear();
        BT_environment.obstacles_Y2.clear();

        BT_environment.Initial_objects_X.clear();
        BT_environment.Initial_objects_Y.clear();
        BT_environment.objects_X.clear();
        BT_environment.objects_Y.clear();
        BT_environment.objects_picked.clear();

        BT_environment.Initial_swarm_X.clear();
        BT_environment.Initial_swarm_Y.clear();

    }

    void generate_env() {


        for (int i = 0; i < BT.size(); i++) {
            if (BT[i].node == 10 || BT[i].node == 11
                || BT[i].node == 16 || BT[i].node == 17 ) {
                BT_environment.areas_in_env = true;

            }

            if (BT[i].node == 18 || BT[i].node == 19 || BT[i].node == 23) {
                BT_environment.objects_in_env = true;

            }

            if (BT[i].node == 20) {
                BT_environment.obstacles_in_env = true;

            }
        }

        //    blackboard.BT_Env=  blackboard::env();
        if (BT_environment.areas_in_env) {
            double x, y, w;
            int area_min_size = 100;
            bool Area_Searching, no_overlap = true;
            BT_environment.areas_num = (rand() % 6) + 1;

           // BT_environment.areas_X.resize(BT_environment.areas_num);
          //  BT_environment.areas_Y.resize(BT_environment.areas_num);
           // BT_environment.areas_Width.resize(BT_environment.areas_num);
            w = rand() % ((max / 2) - area_min_size + 1) + area_min_size;// min area size =50
            x = rand() % ((max-int(w)) - min + 1) + min;
            y = rand() % ((max-int(w)) - min + 1) + min;


            BT_environment.areas_X.push_back(x);
            BT_environment.areas_Y.push_back(y);
            BT_environment.areas_Width.push_back(w);
            for (int i = 1; i < BT_environment.areas_num; i++) {
                Area_Searching = true;
                while (Area_Searching) {
                    w = rand() % ((max / 2) - area_min_size + 1) + area_min_size;// min area size =50
                    x = rand() % ((max-int(w)) - min + 1) + min;
                    y = rand() % ((max-int(w)) - min + 1) + min;

                    no_overlap = true;
                    for (int j = 0; j < BT_environment.areas_X.size(); j++) {
                        if ((x + w) > BT_environment.areas_X[j] and
                            (y + w) > BT_environment.areas_Y[j] and
                            x < (BT_environment.areas_X[j] + BT_environment.areas_Width[j]) and
                            y < (BT_environment.areas_Y[j] + BT_environment.areas_Width[j])) {
                            no_overlap = no_overlap and false;
                        }
                    }
                    if (no_overlap and (x + w) < max and (y + w) < max) {
                        BT_environment.areas_X.push_back(x);
                        BT_environment.areas_Y.push_back(y);
                        BT_environment.areas_Width.push_back(w);
                        Area_Searching = false;
                    }

                }

            }

             BT_environment.areas_X.resize(BT_environment.areas_num);
              BT_environment.areas_Y.resize(BT_environment.areas_num);
             BT_environment.areas_Width.resize(BT_environment.areas_num);

        }

        if (BT_environment.obstacles_in_env) {

           // BT_environment.obstacles_num = (rand() % 4) + 1;

            BT_environment.obstacles_num= (rand() % 3) + 1;




            for (int i = 0; i < BT_environment.obstacles_num; i++) {
                float length = (rand() % (200 - 150 + 1) + 150); // length of the obstacle , min = 150 , max = 200;
                float angle =  ((double) rand() / (RAND_MAX)) * (2.0 * M_PI);
                float x1 = rand() % (max - min + 1) + min;
                float y1 = rand() % (max - min + 1) + min;
                float x2 = x1 + length * cos(angle);
                float y2 = y1 + length * sin(angle);

                //int min_t = -50+(100/(BT_environment.obstacles_num))*i;
                //int max_t = -50+(100/(BT_environment.obstacles_num))*(i+1);


                BT_environment.obstacles_X.push_back(x1);
                BT_environment.obstacles_Y.push_back(y1);
                BT_environment.obstacles_X2.push_back(x2);
                BT_environment.obstacles_Y2.push_back(y2);

            }

              BT_environment.obstacles_X.resize(BT_environment.obstacles_num);
              BT_environment.obstacles_Y.resize(BT_environment.obstacles_num);
             BT_environment.obstacles_X2.resize(BT_environment.obstacles_num);
             BT_environment.obstacles_Y2.resize(BT_environment.obstacles_num);

        }

        if (BT_environment.objects_in_env) {
            BT_environment.objects_num = (rand() % 21) + 5;
            for (int i = 0; i < BT_environment.objects_num; i++) {
                BT_environment.objects_X.push_back(rand() % ((max - 2) - (min + 2) + 1) + (min + 2));
                BT_environment.objects_Y.push_back(rand() % ((max - 2) - (min + 2) + 1) + (min + 2));
                BT_environment.objects_picked.push_back(false);
            }
            BT_environment.Initial_objects_X = BT_environment.objects_X;
            BT_environment.Initial_objects_Y = BT_environment.objects_Y;
            BT_environment.objects_X.resize(BT_environment.objects_num);
            BT_environment.objects_Y.resize(BT_environment.objects_num);
            BT_environment.objects_picked.resize(BT_environment.objects_num);
        }
    }

    void generate_env(bool areas_env,bool objects_env) {

        BT_environment.areas_in_env = areas_env;
        BT_environment.objects_in_env = objects_env;


        //    blackboard.BT_Env=  blackboard::env();
        if (BT_environment.areas_in_env) {
            double x, y, w;
            int area_min_size = 100;
            bool Area_Searching, no_overlap = true;
            BT_environment.areas_num = (rand() % 6) + 1;;

            // BT_environment.areas_X.resize(BT_environment.areas_num);
            //  BT_environment.areas_Y.resize(BT_environment.areas_num);
            // BT_environment.areas_Width.resize(BT_environment.areas_num);
            w = rand() % ((max / 2) - area_min_size + 1) + area_min_size;// min area size =50
            x = rand() % ((max-int(w)) - min + 1) + min;
            y = rand() % ((max-int(w)) - min + 1) + min;


            BT_environment.areas_X.push_back(x);
            BT_environment.areas_Y.push_back(y);
            BT_environment.areas_Width.push_back(w);
            for (int i = 1; i < BT_environment.areas_num; i++) {
                Area_Searching = true;
                while (Area_Searching) {
                    w = rand() % ((max / 2) - area_min_size + 1) + area_min_size;// min area size =50
                    x = rand() % ((max-int(w)) - min + 1) + min;
                    y = rand() % ((max-int(w)) - min + 1) + min;

                    no_overlap = true;
                    for (int j = 0; j < BT_environment.areas_X.size(); j++) {
                        if ((x + w) > BT_environment.areas_X[j] and
                            (y + w) > BT_environment.areas_Y[j] and
                            x < (BT_environment.areas_X[j] + BT_environment.areas_Width[j]) and
                            y < (BT_environment.areas_Y[j] + BT_environment.areas_Width[j])) {
                            no_overlap = no_overlap and false;
                        }
                    }
                    if (no_overlap and (x + w) < max and (y + w) < max) {
                        BT_environment.areas_X.push_back(x);
                        BT_environment.areas_Y.push_back(y);
                        BT_environment.areas_Width.push_back(w);
                        Area_Searching = false;
                    }

                }

            }

            BT_environment.areas_X.resize(BT_environment.areas_num);
            BT_environment.areas_Y.resize(BT_environment.areas_num);
            BT_environment.areas_Width.resize(BT_environment.areas_num);

        }


        if (BT_environment.objects_in_env) {
            BT_environment.objects_num = (rand() % 21) + 5;;
            for (int i = 0; i < BT_environment.objects_num; i++) {
                BT_environment.objects_X.push_back(rand() % ((max - 2) - (min + 2) + 1) + (min + 2));
                BT_environment.objects_Y.push_back(rand() % ((max - 2) - (min + 2) + 1) + (min + 2));
                BT_environment.objects_picked.push_back(false);
            }
            BT_environment.Initial_objects_X = BT_environment.objects_X;
            BT_environment.Initial_objects_Y = BT_environment.objects_Y;
            BT_environment.objects_X.resize(BT_environment.objects_num);
            BT_environment.objects_Y.resize(BT_environment.objects_num);
            BT_environment.objects_picked.resize(BT_environment.objects_num);
        }
    }

    void draw_env() {


        for (int i = 0; i < BT.size(); i++) {
            if (BT[i].node == 10 || BT[i].node == 11
                || BT[i].node == 16 || BT[i].node == 17 ) {
                BT_environment.areas_in_env = true;

            }

            if (BT[i].node == 18 || BT[i].node == 19) {
                BT_environment.objects_in_env = true;

            }

            if (BT[i].node == 20) {
                BT_environment.obstacles_in_env = true;

            }
        }


        if (BT_environment.areas_in_env) {

        BT_environment.areas_num=5;

        BT_environment.areas_X.push_back(-250);
        BT_environment.areas_Y.push_back(-250);
        BT_environment.areas_Width.push_back(100);

        BT_environment.areas_X.push_back(150);
        BT_environment.areas_Y.push_back(-250);
        BT_environment.areas_Width.push_back(100);

        BT_environment.areas_X.push_back(-250);
        BT_environment.areas_Y.push_back(150);
        BT_environment.areas_Width.push_back(100);

        BT_environment.areas_X.push_back(150);
        BT_environment.areas_Y.push_back(150);
        BT_environment.areas_Width.push_back(100);

        BT_environment.areas_X.push_back(-75);
        BT_environment.areas_Y.push_back(-75);
        BT_environment.areas_Width.push_back(150);


        BT_environment.areas_X.resize(BT_environment.areas_num);
        BT_environment.areas_Y.resize(BT_environment.areas_num);
        BT_environment.areas_Width.resize(BT_environment.areas_num);

        }

        if (BT_environment.obstacles_in_env) {


            BT_environment.obstacles_num= 2;

            float x1 = -225 ;
            float y1 = 0;
            float x2 = x1 + 200 * cos(0.8);
            float y2 = y1 + 200 * sin(0.8);

            BT_environment.obstacles_X.push_back(x1);
            BT_environment.obstacles_Y.push_back(y1);
            BT_environment.obstacles_X2.push_back(x2);
            BT_environment.obstacles_Y2.push_back(y2);

            x1 = 25;
            y1 = -225;
            x2 = x1 + 200 * cos(0.8);
            y2 = y1 + 200 * sin(0.8);

            BT_environment.obstacles_X.push_back(x1);
            BT_environment.obstacles_Y.push_back(y1);
            BT_environment.obstacles_X2.push_back(x2);
            BT_environment.obstacles_Y2.push_back(y2);



            BT_environment.obstacles_X.resize(BT_environment.obstacles_num);
            BT_environment.obstacles_Y.resize(BT_environment.obstacles_num);
            BT_environment.obstacles_X2.resize(BT_environment.obstacles_num);
            BT_environment.obstacles_Y2.resize(BT_environment.obstacles_num);

        }

        if (BT_environment.objects_in_env) {
            BT_environment.objects_num = 15;

            // objects inside and around middle area
            BT_environment.objects_X.push_back(-50);
            BT_environment.objects_Y.push_back(-25);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(25);
            BT_environment.objects_Y.push_back(0);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(0);
            BT_environment.objects_Y.push_back(-110);
            BT_environment.objects_picked.push_back(false);

            // objects inside and around bottom left area
            BT_environment.objects_X.push_back(-130);
            BT_environment.objects_Y.push_back(-200);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(-200);
            BT_environment.objects_Y.push_back(-185);
            BT_environment.objects_picked.push_back(false);

            // objects inside and around bottom right area
            BT_environment.objects_X.push_back(225);
            BT_environment.objects_Y.push_back(-200);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(135);
            BT_environment.objects_Y.push_back(-175);
            BT_environment.objects_picked.push_back(false);

            // objects inside and around top right area
            BT_environment.objects_X.push_back(185);
            BT_environment.objects_Y.push_back(185);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(160);
            BT_environment.objects_Y.push_back(120);
            BT_environment.objects_picked.push_back(false);

            // objects inside and around top left area
            BT_environment.objects_X.push_back(-170);
            BT_environment.objects_Y.push_back(180);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(-125);
            BT_environment.objects_Y.push_back(175);
            BT_environment.objects_picked.push_back(false);

            // scattered objects
            BT_environment.objects_X.push_back(10);
            BT_environment.objects_Y.push_back(-160);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(-10);
            BT_environment.objects_Y.push_back(180);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(180);
            BT_environment.objects_Y.push_back(10);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(-160);
            BT_environment.objects_Y.push_back(-10);
            BT_environment.objects_picked.push_back(false);



            BT_environment.Initial_objects_X = BT_environment.objects_X;
            BT_environment.Initial_objects_Y = BT_environment.objects_Y;
            BT_environment.objects_X.resize(BT_environment.objects_num);
            BT_environment.objects_Y.resize(BT_environment.objects_num);
            BT_environment.objects_picked.resize(BT_environment.objects_num);
        }
    }


    void draw_all_env() {



     BT_environment.areas_in_env = true;
     BT_environment.objects_in_env = true;

        if (BT_environment.areas_in_env) {

            BT_environment.areas_num=5;

            BT_environment.areas_X.push_back(-250);
            BT_environment.areas_Y.push_back(-250);
            BT_environment.areas_Width.push_back(100);

            BT_environment.areas_X.push_back(150);
            BT_environment.areas_Y.push_back(-250);
            BT_environment.areas_Width.push_back(100);

            BT_environment.areas_X.push_back(-250);
            BT_environment.areas_Y.push_back(150);
            BT_environment.areas_Width.push_back(100);

            BT_environment.areas_X.push_back(150);
            BT_environment.areas_Y.push_back(150);
            BT_environment.areas_Width.push_back(100);

            BT_environment.areas_X.push_back(-75);
            BT_environment.areas_Y.push_back(-75);
            BT_environment.areas_Width.push_back(150);


            BT_environment.areas_X.resize(BT_environment.areas_num);
            BT_environment.areas_Y.resize(BT_environment.areas_num);
            BT_environment.areas_Width.resize(BT_environment.areas_num);

        }


        if (BT_environment.objects_in_env) {
            BT_environment.objects_num = 15;

            // objects inside and around middle area
            BT_environment.objects_X.push_back(-50);
            BT_environment.objects_Y.push_back(-25);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(25);
            BT_environment.objects_Y.push_back(0);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(0);
            BT_environment.objects_Y.push_back(-110);
            BT_environment.objects_picked.push_back(false);

            // objects inside and around bottom left area
            BT_environment.objects_X.push_back(-130);
            BT_environment.objects_Y.push_back(-200);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(-200);
            BT_environment.objects_Y.push_back(-185);
            BT_environment.objects_picked.push_back(false);

            // objects inside and around bottom right area
            BT_environment.objects_X.push_back(225);
            BT_environment.objects_Y.push_back(-200);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(135);
            BT_environment.objects_Y.push_back(-175);
            BT_environment.objects_picked.push_back(false);

            // objects inside and around top right area
            BT_environment.objects_X.push_back(185);
            BT_environment.objects_Y.push_back(185);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(160);
            BT_environment.objects_Y.push_back(120);
            BT_environment.objects_picked.push_back(false);

            // objects inside and around top left area
            BT_environment.objects_X.push_back(-170);
            BT_environment.objects_Y.push_back(180);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(-125);
            BT_environment.objects_Y.push_back(175);
            BT_environment.objects_picked.push_back(false);

            // scattered objects
            BT_environment.objects_X.push_back(10);
            BT_environment.objects_Y.push_back(-160);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(-10);
            BT_environment.objects_Y.push_back(180);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(180);
            BT_environment.objects_Y.push_back(10);
            BT_environment.objects_picked.push_back(false);
            BT_environment.objects_X.push_back(-160);
            BT_environment.objects_Y.push_back(-10);
            BT_environment.objects_picked.push_back(false);



            BT_environment.Initial_objects_X = BT_environment.objects_X;
            BT_environment.Initial_objects_Y = BT_environment.objects_Y;
            BT_environment.objects_X.resize(BT_environment.objects_num);
            BT_environment.objects_Y.resize(BT_environment.objects_num);
            BT_environment.objects_picked.resize(BT_environment.objects_num);
        }
    }


    void read_env (const std::vector<std::vector<double>>& objectsX, const std::vector<std::vector<double>>& objectsY, int objNum)
    {
        BT_environment.Initial_objects_X.clear();
        BT_environment.Initial_objects_Y.clear();
        BT_environment.objects_in_env=true;
        BT_environment.objects_Data_X=objectsX;
        BT_environment.objects_Data_Y=objectsY;
        BT_environment.objects_num=objNum;
        BT_environment.Initial_objects_X=objectsX[0];
        BT_environment.Initial_objects_Y=objectsY[0];
        for (int b=0;b<objNum;b++)
        {
            BT_environment.objects_picked.push_back(false);
        }




        BT_environment.areas_X.clear();
        BT_environment.areas_Y.clear();
        BT_environment.areas_Width.clear();
        BT_environment.areas_in_env=true;
       BT_environment.areas_num=1;
        BT_environment.areas_X.push_back(-100);
        BT_environment.areas_Y.push_back(-100);
        BT_environment.areas_Width.push_back(200);
   }

    void read_env2 (const std::vector<std::vector<double>>& objectsX, const std::vector<std::vector<double>>& objectsY, int objNum)
    {
        BT_environment.Initial_objects_X.clear();
        BT_environment.Initial_objects_Y.clear();
        BT_environment.objects_in_env=true;
        BT_environment.areas_in_env=true;
        if (BT_environment.objects_in_env)
        {
            BT_environment.objects_Data_X=objectsX;
            BT_environment.objects_Data_Y=objectsY;
            BT_environment.objects_num=objNum;
            BT_environment.Initial_objects_X=objectsX[0];
            BT_environment.Initial_objects_Y=objectsY[0];
            for (int b=0;b<objNum;b++)
            {
                BT_environment.objects_picked.push_back(false);
            }
        }
        if(BT_environment.areas_in_env)
        {
            BT_environment.areas_num=5;

            BT_environment.areas_X.push_back(-250);
            BT_environment.areas_Y.push_back(-250);
            BT_environment.areas_Width.push_back(100);

            BT_environment.areas_X.push_back(150);
            BT_environment.areas_Y.push_back(-250);
            BT_environment.areas_Width.push_back(100);

            BT_environment.areas_X.push_back(-250);
            BT_environment.areas_Y.push_back(150);
            BT_environment.areas_Width.push_back(100);

            BT_environment.areas_X.push_back(150);
            BT_environment.areas_Y.push_back(150);
            BT_environment.areas_Width.push_back(100);

            BT_environment.areas_X.push_back(-75);
            BT_environment.areas_Y.push_back(-75);
            BT_environment.areas_Width.push_back(150);


            BT_environment.areas_X.resize(BT_environment.areas_num);
            BT_environment.areas_Y.resize(BT_environment.areas_num);
            BT_environment.areas_Width.resize(BT_environment.areas_num);
        }






    }

    void generate_swarm ()
    {
        for (int i=0;i< blackboard::Swarm_size;i++)
        {
            BT_environment.Initial_swarm_X.push_back(rand()%(max-min+1)+min);
            BT_environment.Initial_swarm_Y.push_back(rand()%(max-min+1)+min);
        }
    }

    void generate_env_areas() {

        BT_environment.areas_X.clear();
        BT_environment.areas_Y.clear();
        BT_environment.areas_Width.clear();



        //    blackboard.BT_Env=  blackboard::env();
            double x, y, w;
            int area_min_size = 100;
            bool Area_Searching, no_overlap = true;
            BT_environment.areas_num = (rand() % 5) + 1;

            // BT_environment.areas_X.resize(BT_environment.areas_num);
            //  BT_environment.areas_Y.resize(BT_environment.areas_num);
            // BT_environment.areas_Width.resize(BT_environment.areas_num);
            w = rand() % ((max / 2) - area_min_size + 1) + area_min_size;// min area size =50
            x = rand() % ((max-int(w)) - min + 1) + min;
            y = rand() % ((max-int(w)) - min + 1) + min;


            BT_environment.areas_X.push_back(x);
            BT_environment.areas_Y.push_back(y);
            BT_environment.areas_Width.push_back(w);
            for (int i = 1; i < BT_environment.areas_num; i++) {
                Area_Searching = true;
                while (Area_Searching) {
                    w = rand() % ((max / 2) - area_min_size + 1) + area_min_size;// min area size =50
                    x = rand() % ((max-int(w)) - min + 1) + min;
                    y = rand() % ((max-int(w)) - min + 1) + min;

                    no_overlap = true;
                    for (int j = 0; j < BT_environment.areas_X.size(); j++) {
                        if ((x + w) > BT_environment.areas_X[j] and
                            (y + w) > BT_environment.areas_Y[j] and
                            x < (BT_environment.areas_X[j] + BT_environment.areas_Width[j]) and
                            y < (BT_environment.areas_Y[j] + BT_environment.areas_Width[j])) {
                            no_overlap = no_overlap and false;
                        }
                    }
                    if (no_overlap and (x + w) < max and (y + w) < max) {
                        BT_environment.areas_X.push_back(x);
                        BT_environment.areas_Y.push_back(y);
                        BT_environment.areas_Width.push_back(w);
                        Area_Searching = false;
                    }

                }

            }

            BT_environment.areas_X.resize(BT_environment.areas_num);
            BT_environment.areas_Y.resize(BT_environment.areas_num);
            BT_environment.areas_Width.resize(BT_environment.areas_num);




    }
    void generate_env_objects() {



        BT_environment.Initial_objects_X.clear();
        BT_environment.Initial_objects_Y.clear();
        BT_environment.objects_X.clear();
        BT_environment.objects_Y.clear();
        BT_environment.objects_picked.clear();


            BT_environment.objects_num = (rand() % 16) + 5;
            for (int i = 0; i < BT_environment.objects_num; i++) {
                BT_environment.objects_X.push_back(rand() % ((max - 2) - (min + 2) + 1) + (min + 2));
                BT_environment.objects_Y.push_back(rand() % ((max - 2) - (min + 2) + 1) + (min + 2));
                BT_environment.objects_picked.push_back(false);
            }

            BT_environment.Initial_objects_X = BT_environment.objects_X;
            BT_environment.Initial_objects_Y = BT_environment.objects_Y;
            BT_environment.objects_X.resize(BT_environment.objects_num);
            BT_environment.objects_Y.resize(BT_environment.objects_num);
            BT_environment.objects_picked.resize(BT_environment.objects_num);



    }
    void generate_env_obstacles() {


        BT_environment.obstacles_X.clear();
        BT_environment.obstacles_Y.clear();
        BT_environment.obstacles_X2.clear();
        BT_environment.obstacles_Y2.clear();





            // BT_environment.obstacles_num = (rand() % 4) + 1;

            BT_environment.obstacles_num= (rand() % 3) + 1;




            for (int i = 0; i < BT_environment.obstacles_num; i++) {
                float length = (rand() % (200 - 150 + 1) + 150); // length of the obstacle , min = 150 , max = 200;
                float angle =  ((double) rand() / (RAND_MAX)) * (2.0 * M_PI);
                float x1 = rand() % (max - min + 1) + min;
                float y1 = rand() % (max - min + 1) + min;
                float x2 = x1 + length * cos(angle);
                float y2 = y1 + length * sin(angle);

                //int min_t = -50+(100/(BT_environment.obstacles_num))*i;
                //int max_t = -50+(100/(BT_environment.obstacles_num))*(i+1);


                BT_environment.obstacles_X.push_back(x1);
                BT_environment.obstacles_Y.push_back(y1);
                BT_environment.obstacles_X2.push_back(x2);
                BT_environment.obstacles_Y2.push_back(y2);

            }

            BT_environment.obstacles_X.resize(BT_environment.obstacles_num);
            BT_environment.obstacles_Y.resize(BT_environment.obstacles_num);
            BT_environment.obstacles_X2.resize(BT_environment.obstacles_num);
            BT_environment.obstacles_Y2.resize(BT_environment.obstacles_num);




    }

    };





#endif //MAIN_CPP_BEHAVIORTREE_H
