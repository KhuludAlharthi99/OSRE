

#include "rapidcsv.h"
#include <memory>
#include "SwarmMetrics.h"
#include "Model2.h"
#include <vector>
#include <numeric>
#include "BTevolution_IslandModel.h"
#include "Env_Evolution.h"




#ifndef MAIN_CPP_MODELTEST_H
#define MAIN_CPP_MODELTEST_H

class ModelTest {

public:
    int test_size;
    vector<vector <blackboard::Node>> original_Bts;
    vector<vector <blackboard::Node>> extracted_Bts;

    vector<vector <blackboard::Node>> original_Bts_r;
    vector<vector <blackboard::Node>> extracted_Bts_r;

    vector <vector<double>> Best_fitness_All;
    vector <vector<double>> Avg_fitness_All;
    vector<blackboard::Node> BT;
    int sub_root;
    int subtree_size;
    int final_child=10 ;
    int node_index = 0;
    int removed;

    vector<int> sub_tree_nodes ={};
    vector<int> sub_tree_nodes_index ={};
    std::map< int, int > Seq_type;
    bool pruned = true;
    vector<pair<int,vector<int>>> sub_trees;
    vector<int> subtree_temp;
    int child_num_temp,child_num_temp2,child_num_temp3 ;
    unique_ptr<Swarm> sm;


    vector<int> BT_nodes = {0,1,2,3,4,5,6,7,8,13,16,17,18,19,23};
    vector<int> nodes_list = {2,3,4,5,6,7,8,9,13,16,17,18,19,23};

    //vector<int> BT_nodes = {0,1,2,3,4,5,6,7,8,9,13,10,16,17};
    //vector<int> nodes_list = {2,3,4,5,6,7,8,9,13,10,16,17};

    // objects nodes : 18,18,18,19,23,24
    //areas nodes : 10,11,16,17

    // all nodes
    //vector<int> action_Leaf_nodes_level1 = {4,5,6,7,8,10,11};
    //vector<int> action_Leaf_nodes_level2 = {2,3,9,18,20,21,23,24};
    //vector<int> Leaf_nodes_level2 = {2,3,9,12,13,14,15,16,17,18,20,21,22,23,24};
    //vector<int> condition_Leaf_nodes = {12,13,14,15,16,17,22};

   /* //All Nodes
    vector<int> action_Leaf_nodes0 = {2,3,4,5,6,7,8,9,10,11,18,19,20,21}; //add env based actions
    vector<int> all_leaf_nodes0 = {2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22}; //add env based actions and conditions
    vector<int> condition_Leaf_nodes0 = {12,13,14,15,16,17,22}; //add env based conditions


    vector<int> action_Leaf_nodes = {2,3,4,5,6,7,8,9,10,11,18,19,20,20,21}; //add env based actions
    vector<int> all_leaf_nodes = {2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22}; //add env based actions and conditions
    vector<int> condition_Leaf_nodes = {12,13,14,15,16,17,22}; //add env based conditions*/

    // Motion nodes
    vector<int> action_Leaf_nodes0 = {2,3,4,5,6,7,8,9,18,18,18,19,19}; //add env based actions
    vector<int> all_leaf_nodes0 = {2,3,4,5,6,7,8,9,13,18,18,18,19,19,23}; //add env based actions and conditions
    vector<int> condition_Leaf_nodes0 = {13,23}; //add env based conditions

    vector<int> action_Leaf_nodes = {2,3,4,5,6,7,8,9,18,18,18,19,19}; //add env based actions
    vector<int> all_leaf_nodes = {2,3,4,5,6,7,8,9,13,18,18,18,19,19,23}; //add env based actions and conditions
    vector<int> condition_Leaf_nodes = {13,23}; //add env based conditions


    // area conditions 10,10,11,11,16,16,17,17
    // object actions 18,19,18,19
    // object conditions 14,14,23,23,24,24
    // object nodes 14,18,18,19,23,24
    int send_msg_node=0;
    int read_msg_node=0;
    int drop_object_node =0;
    int last_added_node =0;

    std::map< int, int > BT_Nodes_tick_times;







    ModelTest( int testSize) {
        test_size = testSize;

    }



    void Add_leaf_node ()
    {
        int random_index = (rand() % all_leaf_nodes.size());
        BT.push_back({all_leaf_nodes[random_index], 0});

        if(all_leaf_nodes[random_index]==19 or
           all_leaf_nodes[random_index]==14 or
           all_leaf_nodes[random_index]==23 or
           all_leaf_nodes[random_index]==24)
        {
            all_leaf_nodes.push_back(18);
        }

        if(all_leaf_nodes[random_index]==22)
        {
            all_leaf_nodes.push_back(21);
        }
    }

    void Add_leaf_node (int node)
    {
        BT.push_back({node, 0});

        if(node==19 or
           node==14 or
           node==23 or
           node==24)
        {
            all_leaf_nodes.push_back(18);
        }

        if(node==22)
        {
            all_leaf_nodes.push_back(21);
        }
    }

    void Add_leaf_node2 ()
    {
        if (sub_root ==0) //Seq
        {
            if (final_child ==0)
            {
                //int random_index = generateRandomNumber(0,action_Leaf_nodes.size()-1);
                int random_index = (rand() % action_Leaf_nodes.size());
                BT.push_back({action_Leaf_nodes[random_index], 0});
                last_added_node=action_Leaf_nodes[random_index];


                if(action_Leaf_nodes[random_index]==19 or
                   action_Leaf_nodes[random_index]==14 or
                   action_Leaf_nodes[random_index]==23 or
                   action_Leaf_nodes[random_index]==24)
                {
                    drop_object_node=1;
                    action_Leaf_nodes.push_back(18);
                    all_leaf_nodes.push_back(18);

                }
                if(action_Leaf_nodes[random_index]==21)
                {
                    send_msg_node=1;

                }
            }
            else
            {
                if (send_msg_node==1 && read_msg_node==0  && last_added_node!=21)
                {
                    all_leaf_nodes.push_back(22);
                    condition_Leaf_nodes.push_back(22);
                }

                if (drop_object_node ==1)
                {
                    int random_index = (rand() % condition_Leaf_nodes.size());
                    //int random_index = generateRandomNumber(0,condition_Leaf_nodes.size()-1);
                    BT.push_back({condition_Leaf_nodes[random_index], 0});
                    last_added_node=condition_Leaf_nodes[random_index];

                    if(condition_Leaf_nodes[random_index]==22)
                    {
                        read_msg_node=1;

                    }
                    drop_object_node=0;
                }
                else
                {
                    int random_index = (rand() % all_leaf_nodes.size());
                    //int random_index = generateRandomNumber(0,all_leaf_nodes.size()-1);
                    BT.push_back({all_leaf_nodes[random_index], 0});
                    last_added_node=all_leaf_nodes[random_index];



                    if(all_leaf_nodes[random_index]==19 or
                       all_leaf_nodes[random_index]==14 or
                       all_leaf_nodes[random_index]==23 or
                       all_leaf_nodes[random_index]==24)
                    {
                        drop_object_node=1;
                        action_Leaf_nodes.push_back(18);
                        all_leaf_nodes.push_back(18);

                    }
                    if(all_leaf_nodes[random_index]==21)
                    {
                        send_msg_node=1;

                    }
                    if(all_leaf_nodes[random_index]==22)
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
                    int random_index = (rand() % action_Leaf_nodes.size());
                    //int random_index = generateRandomNumber(0,action_Leaf_nodes.size()-1);
                    BT.push_back({action_Leaf_nodes[random_index], 0});
                    last_added_node=action_Leaf_nodes[random_index];




                  if(action_Leaf_nodes[random_index]==19
                     or action_Leaf_nodes[random_index]==14
                     or action_Leaf_nodes[random_index]==23
                     or action_Leaf_nodes[random_index]==24)
                   {
                      action_Leaf_nodes.push_back(18);
                     all_leaf_nodes.push_back(18);

                   }
                    if(action_Leaf_nodes[random_index]==21)
                    {
                        send_msg_node=1;

                    }

            }
            else
            {

                    if (send_msg_node==1 && read_msg_node==0 && last_added_node!=21)
                    {
                        all_leaf_nodes.push_back(22);
                        condition_Leaf_nodes.push_back(22);
                    }

                    int random_index = (rand() % condition_Leaf_nodes.size());
                    //int random_index = generateRandomNumber(0,condition_Leaf_nodes.size()-1);
                    BT.push_back({condition_Leaf_nodes[random_index], 0});
                    last_added_node=condition_Leaf_nodes[random_index];

                if(condition_Leaf_nodes[random_index]==22)
                {
                    read_msg_node=1;

                }



                }

            }
    }

    void Full(int depth, int max_depth, int breadth) {
        if (depth == max_depth) {
            Add_leaf_node2();
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
            Add_leaf_node2();
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
                    Add_leaf_node2();
                    node_index += 1;
                    return;
                }

            }
        }}

    void RandomBT_generator(int depth) {
        action_Leaf_nodes = action_Leaf_nodes0;
        all_leaf_nodes = all_leaf_nodes0;
        condition_Leaf_nodes = condition_Leaf_nodes0;
        send_msg_node=0;
        read_msg_node=0;
        drop_object_node=0;
        last_added_node=0;
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

    vector<blackboard::Node> decode_prune0 (vector<blackboard::Node> tree , int index)
    {
        int child_index;
        switch(tree[index].node){
            case 2 ... 24:
                sub_tree_nodes.push_back(tree[index].node);
                sub_tree_nodes_index.push_back(index);
                index = index -1;
                return decode_prune0(tree,index);
            case 0:
                child_num_temp = tree[index].child_num;
                child_index = sub_tree_nodes.size()-child_num_temp;
                for(int i =0; i<child_num_temp;i++)
                    {
                        if(sub_tree_nodes[child_index+i]== 0)
                        {
                            tree[index].child_num+=tree[sub_tree_nodes_index[child_index+i]].child_num-1;
                            tree[sub_tree_nodes_index[child_index+i]].node = 100;
                            //tree.erase(tree.begin()+sub_tree_nodes_index[child_index+i]);
                            //update_indexes_vector(sub_tree_nodes_index[child_index+i]);

                            pruned = true;
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
                    return decode_prune0(tree,index);
                }
            case 1:
                child_num_temp = tree[index].child_num;
                child_index = sub_tree_nodes.size()-child_num_temp;
                for(int i =0; i<child_num_temp;i++)
                    {


                        if(sub_tree_nodes[child_index+i]== 1)
                        {
                            tree[index].child_num+=tree[sub_tree_nodes_index[child_index+i]].child_num-1;
                            tree[sub_tree_nodes_index[child_index+i]].node = 100;
                            //tree.erase(tree.begin()+sub_tree_nodes_index[child_index+i]);
                            //update_indexes_vector(sub_tree_nodes_index[child_index+i]);

                            pruned = true;
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
                    return decode_prune0(tree,index);
                }
        }
    }

    vector<blackboard::Node> decode_prune1 (vector<blackboard::Node> tree , int index)
    {
        switch(tree[index].node){
            case 2 ... 24:
                sub_tree_nodes.push_back(tree[index].node);
                sub_tree_nodes_index.push_back(index);
                index = index -1;
                return decode_prune1(tree,index);
            case 0:
                child_num_temp = tree[index].child_num;
                if (tree[index].child_num==1 && index>0)
                {
                    tree[index].child_num=tree[sub_tree_nodes_index.back()].child_num;
                    tree[index].node= tree[sub_tree_nodes_index.back()].node;
                    tree[sub_tree_nodes_index.back()].node = 100;
                    //tree.erase(tree.begin()+sub_tree_nodes_index.back());
                    //update_indexes_vector(sub_tree_nodes_index.back());
                    pruned = true;

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
                    return decode_prune1(tree,index);
                }
            case 1:
                child_num_temp = tree[index].child_num;
                if (tree[index].child_num==1 &&  index>0)
                {
                    tree[index].child_num=tree[sub_tree_nodes_index.back()].child_num;
                    tree[index].node= tree[sub_tree_nodes_index.back()].node;
                    tree[sub_tree_nodes_index.back()].node = 100;
                    //tree.erase(tree.begin()+sub_tree_nodes_index.back());
                    //update_indexes_vector(sub_tree_nodes_index.back());

                    pruned = true;

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
                    return decode_prune1(tree,index);
                }
        }
    }

    vector<blackboard::Node> decode_prune2 (vector<blackboard::Node> tree , int index)
    {
        int actions_num;
        int child_index;
        switch(tree[index].node){
            case 2 ... 24:
                sub_tree_nodes.push_back(tree[index].node);
                sub_tree_nodes_index.push_back(index);
                index = index -1;
                return decode_prune2(tree,index);
            case 0:
                actions_num=0;
                child_num_temp = tree[index].child_num;
                child_index = sub_tree_nodes.size()-child_num_temp;
                for (int i=0; i<child_num_temp ;i++)
                {
                    if(sub_tree_nodes[child_index+i]!= 10 && sub_tree_nodes[child_index+i]!= 11 && sub_tree_nodes[child_index+i]!= 12
                       && sub_tree_nodes[child_index+i]!= 13 && sub_tree_nodes[child_index+i]!= 14 && sub_tree_nodes[child_index+i]!= 15
                       && sub_tree_nodes[child_index+i]!= 16 && sub_tree_nodes[child_index+i]!= 17 && sub_tree_nodes[child_index+i]!= 22
                       && sub_tree_nodes[child_index+i]!= 23 && sub_tree_nodes[child_index+i]!= 24
                       && sub_tree_nodes[child_index+i]!= 0 && sub_tree_nodes[child_index+i]!= 1)
                    {
                        actions_num+=1;
                    }
                    sub_tree_nodes_index.pop_back();
                    sub_tree_nodes.pop_back();
                }
                if (actions_num == tree[index].child_num)
                {
                    Seq_type[index]=1;
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
                    return decode_prune2(tree,index);
                }
            case 1:
                child_num_temp = tree[index].child_num;
                child_num_temp2 = tree[index].child_num;
                child_index = sub_tree_nodes.size()-child_num_temp;
                for(int i =0; i<child_num_temp;i++)
                {
                    if(sub_tree_nodes[child_index+i]!= 10 && sub_tree_nodes[child_index+i]!= 11 && sub_tree_nodes[child_index+i]!= 12
                       && sub_tree_nodes[child_index+i]!= 13 && sub_tree_nodes[child_index+i]!= 14 && sub_tree_nodes[child_index+i]!= 15
                       && sub_tree_nodes[child_index+i]!= 16 && sub_tree_nodes[child_index+i]!= 17 && sub_tree_nodes[child_index+i]!= 22
                       && sub_tree_nodes[child_index+i]!= 23 && sub_tree_nodes[child_index+i]!= 24
                       && sub_tree_nodes[child_index+i]!= 0 && sub_tree_nodes[child_index+i]!= 1)
                        {
                            if (i+1 < child_num_temp)
                            {
                                child_num_temp3=child_num_temp;
                                for(int j =i+1; j<child_num_temp3;j++)
                                {
                                    if (sub_tree_nodes[child_index+j]==1 || sub_tree_nodes[child_index+j]==0)
                                    {
                                        tree[index].child_num+= tree[sub_tree_nodes_index[child_index+j]].child_num-1;
                                        tree[sub_tree_nodes_index[child_index+j]].node = 100;

                                    }
                                    else
                                    {
                                        tree[index].child_num-=1;
                                        tree[sub_tree_nodes_index[child_index+j]].node = 100;

                                    }
                                    child_num_temp -=1;


                                }
                                pruned = true;
                            }
                        }

                    if( sub_tree_nodes[child_index+i]== 0 && Seq_type[sub_tree_nodes_index[child_index+i]]==1 )
                    {

                        if (i+1 < child_num_temp)
                        {
                            child_num_temp3=child_num_temp;
                            for(int j =i+1; j<child_num_temp3;j++)
                            {
                                if (sub_tree_nodes[child_index+j]==1 || sub_tree_nodes[child_index+j]==0)
                                {
                                    tree[index].child_num+= tree[sub_tree_nodes_index[child_index+j]].child_num-1;
                                    tree[sub_tree_nodes_index[child_index+j]].node = 100;

                                }
                                else
                                {
                                    tree[index].child_num-=1;
                                    tree[sub_tree_nodes_index[child_index+j]].node = 100;

                                }
                                child_num_temp -=1;


                            }
                            pruned = true;
                        }
                    }
                }

                for (int i=0; i<child_num_temp2 ;i++)
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
                    return decode_prune2(tree,index);
                }
        }
    }

    vector<blackboard::Node> decode_prune3 (vector<blackboard::Node> tree , int index,bool pick_obj, bool r_msg,bool s_msg)
    {

        switch(tree[index].node){
            case 2 ... 24:
                sub_tree_nodes.push_back(tree[index].node);
                sub_tree_nodes_index.push_back(index);
                index = index -1;
                return decode_prune3(tree,index,pick_obj,r_msg,s_msg);
            case 0:
                child_num_temp = tree[index].child_num;
                for(int i=0; i<sub_tree_nodes.size();i++) {

                    if (sub_tree_nodes[i] == 19) {
                        if(!pick_obj)
                        {
                            tree[index].child_num-=1;
                            tree.erase(tree.begin()+sub_tree_nodes_index[i]);

                            pruned = true;
                        }
                    }
                    if (sub_tree_nodes[i] == 21) {
                        if(!r_msg)
                        {
                            tree[index].child_num-=1;
                            tree.erase(tree.begin()+sub_tree_nodes_index[i]);
                            pruned = true;
                        }
                    }
                    if (sub_tree_nodes[i] == 22) {
                        if(!s_msg)
                        {
                            tree[index].child_num-=1;
                            tree.erase(tree.begin()+sub_tree_nodes_index[i]);
                            pruned = true;
                        }
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
                    sub_tree_nodes.clear();
                    sub_tree_nodes_index.clear();
                    return tree;
                } else
                {
                    index = index -1;
                    return decode_prune3(tree,index,pick_obj,r_msg,s_msg);
                }
            case 1:
                child_num_temp = tree[index].child_num;
                for(int i=0; i<sub_tree_nodes.size();i++) {

                    if (sub_tree_nodes[i] == 19 or sub_tree_nodes[i] == 14
                        or sub_tree_nodes[i] == 23 or sub_tree_nodes[i] == 24) {
                        if(!pick_obj)
                        {
                            tree[index].child_num-=1;
                            tree.erase(tree.begin()+sub_tree_nodes_index[i]);
                            pruned = true;
                        }
                    }
                    if (sub_tree_nodes[i] == 21) {
                        if(!r_msg)
                        {
                            tree[index].child_num-=1;
                            tree.erase(tree.begin()+sub_tree_nodes_index[i]);
                            pruned = true;
                        }
                    }
                    if (sub_tree_nodes[i] == 22) {
                        if(!s_msg)
                        {
                            tree[index].child_num-=1;
                            tree.erase(tree.begin()+sub_tree_nodes_index[i]);
                            pruned = true;
                        }
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
                    sub_tree_nodes.clear();
                    sub_tree_nodes_index.clear();
                    return tree;
                } else
                {
                    index = index -1;
                    return decode_prune3(tree,index,pick_obj,r_msg,s_msg);
                }
        }
    }

    vector<blackboard::Node> decode_prune4 (vector<blackboard::Node> tree , int index)
    {
        int child_index;
        switch(tree[index].node){
            case 2 ... 24:
                sub_tree_nodes.push_back(tree[index].node);
                sub_tree_nodes_index.push_back(index);
                index = index -1;
                return decode_prune4(tree,index);
            case 0:
                child_num_temp = tree[index].child_num;
                child_index = sub_tree_nodes.size()-child_num_temp;
                for(int i =0; i<child_num_temp-1;i++)
                {
                    if(sub_tree_nodes[child_index+i]== 10 || sub_tree_nodes[child_index+i]== 11 || sub_tree_nodes[child_index+i]== 12
                    || sub_tree_nodes[child_index+i]== 13 || sub_tree_nodes[child_index+i]== 14 || sub_tree_nodes[child_index+i]== 15
                    || sub_tree_nodes[child_index+i]== 16 || sub_tree_nodes[child_index+i]== 17 || sub_tree_nodes[child_index+i]== 22
                    || sub_tree_nodes[child_index+i]== 23 || sub_tree_nodes[child_index+i]== 24)
                    {
                        for (int j = i+1 ; j<child_num_temp ;j++)
                        {
                            if(sub_tree_nodes[child_index+j]== 10 || sub_tree_nodes[child_index+j]== 11 || sub_tree_nodes[child_index+j]== 12
                               || sub_tree_nodes[child_index+j]== 13 || sub_tree_nodes[child_index+j]== 14 || sub_tree_nodes[child_index+j]== 15
                               || sub_tree_nodes[child_index+j]== 16 || sub_tree_nodes[child_index+j]== 17 || sub_tree_nodes[child_index+j]== 22
                               || sub_tree_nodes[child_index+j]== 23 || sub_tree_nodes[child_index+j]== 24)
                            {
                                if (sub_tree_nodes[child_index+i]== sub_tree_nodes[child_index+j])
                                {
                                    tree[index].child_num-=1;
                                    tree[sub_tree_nodes_index[child_index+i]].node = 100;
                                    pruned = true;
                                    break;
                                }

                            }
                            else
                            {
                                break;
                            }
                        }


                    }
                }
                for (int i=0; i<child_num_temp  ;i++)
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
                    return decode_prune4(tree,index);
                }
            case 1:
                child_num_temp = tree[index].child_num;
                child_index = sub_tree_nodes.size()-child_num_temp;
                for(int i =0; i<child_num_temp-1;i++)
                {
                    if(sub_tree_nodes[child_index+i]== 10 || sub_tree_nodes[child_index+i]== 11 || sub_tree_nodes[child_index+i]== 12
                       || sub_tree_nodes[child_index+i]== 13 || sub_tree_nodes[child_index+i]== 14 || sub_tree_nodes[child_index+i]== 15
                       || sub_tree_nodes[child_index+i]== 16 || sub_tree_nodes[child_index+i]== 17 || sub_tree_nodes[child_index+i]== 22
                       || sub_tree_nodes[child_index+i]== 23 || sub_tree_nodes[child_index+i]== 24)
                    {
                        for (int j = i+1 ; j<child_num_temp ;j++)
                        {
                            if(sub_tree_nodes[child_index+j]== 10 || sub_tree_nodes[child_index+j]== 11 || sub_tree_nodes[child_index+j]== 12
                               || sub_tree_nodes[child_index+j]== 13 || sub_tree_nodes[child_index+j]== 14 || sub_tree_nodes[child_index+j]== 15
                               || sub_tree_nodes[child_index+j]== 16 || sub_tree_nodes[child_index+j]== 17 || sub_tree_nodes[child_index+j]== 22
                               || sub_tree_nodes[child_index+j]== 23 || sub_tree_nodes[child_index+j]== 24)
                            {
                                if (sub_tree_nodes[child_index+i]== sub_tree_nodes[child_index+j])
                                {
                                    tree[index].child_num-=1;
                                    tree[sub_tree_nodes_index[child_index+i]].node = 100;
                                    pruned = true;
                                    break;
                                }

                            }
                            else
                            {
                                break;
                            }
                        }


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
                    return decode_prune4(tree,index);
                }
        }
    }



    vector<blackboard::Node> decode_prune5 (vector<blackboard::Node> tree , int index)
    {
        int child_index;
        switch(tree[index].node){
            case 2 ... 24:
                sub_tree_nodes.push_back(tree[index].node);
                sub_tree_nodes_index.push_back(index);
                index = index -1;
                return decode_prune5(tree,index);
            case 0:
                child_num_temp = tree[index].child_num;
                child_index = sub_tree_nodes.size()-child_num_temp;
                for(int i =0; i<child_num_temp-1;i++)
                {
                    switch(sub_tree_nodes[child_index+i]){
                        case 2:
                            removed=0;
                            for (int j = i+1 ; j<child_num_temp ;j++)
                            {
                                if(sub_tree_nodes[child_index+j]== 4 || sub_tree_nodes[child_index+j]== 5
                                   || sub_tree_nodes[child_index+j]== 6 || sub_tree_nodes[child_index+j]== 7
                                   || sub_tree_nodes[child_index+j]== 2 || sub_tree_nodes[child_index+j]== 3
                                   || sub_tree_nodes[child_index+j]== 8 || sub_tree_nodes[child_index+j]== 18
                                   || sub_tree_nodes[child_index+j]== 19 || sub_tree_nodes[child_index+j]== 9)
                                {
                                    if ( sub_tree_nodes[child_index+j]==3)
                                    {
                                        tree[index].child_num-=1;
                                        tree[sub_tree_nodes_index[child_index+j]].node = 100;
                                        sub_tree_nodes[child_index+j]=100;
                                        pruned = true;
                                        removed=1;
                                    }

                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (removed ==1)
                            {
                                tree[index].child_num-=1;
                                tree[sub_tree_nodes_index[child_index+i]].node = 100;
                                sub_tree_nodes[child_index+i]=100;
                            }
                            break;
                        case 3:
                            removed=0;
                            for (int j = i+1 ; j<child_num_temp ;j++)
                            {
                                if(sub_tree_nodes[child_index+j]== 4 || sub_tree_nodes[child_index+j]== 5
                                   || sub_tree_nodes[child_index+j]== 6 || sub_tree_nodes[child_index+j]== 7
                                   || sub_tree_nodes[child_index+j]== 2 || sub_tree_nodes[child_index+j]== 3
                                   || sub_tree_nodes[child_index+j]== 8 || sub_tree_nodes[child_index+j]== 18
                                   || sub_tree_nodes[child_index+j]== 19 || sub_tree_nodes[child_index+j]== 9)
                                {
                                    if ( sub_tree_nodes[child_index+j]==2)
                                    {
                                        tree[index].child_num-=1;
                                        tree[sub_tree_nodes_index[child_index+j]].node = 100;
                                        sub_tree_nodes[child_index+j]=100;
                                        pruned = true;
                                        removed=1;
                                    }

                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (removed ==1)
                            {
                                tree[index].child_num-=1;
                                tree[sub_tree_nodes_index[child_index+i]].node = 100;
                                sub_tree_nodes[child_index+i]=100;
                            }
                            break;
                        case 4:
                            removed=0;
                            for (int j = i+1 ; j<child_num_temp ;j++)
                            {
                                if(sub_tree_nodes[child_index+j]== 4 || sub_tree_nodes[child_index+j]== 5
                                   || sub_tree_nodes[child_index+j]== 6 || sub_tree_nodes[child_index+j]== 7
                                   || sub_tree_nodes[child_index+j]== 2 || sub_tree_nodes[child_index+j]== 3
                                   || sub_tree_nodes[child_index+j]== 8 || sub_tree_nodes[child_index+j]== 18
                                   || sub_tree_nodes[child_index+j]== 19 || sub_tree_nodes[child_index+j]== 9)
                                {
                                    if ( sub_tree_nodes[child_index+j]==6)
                                    {
                                        tree[index].child_num-=1;
                                        tree[sub_tree_nodes_index[child_index+j]].node = 100;
                                        sub_tree_nodes[child_index+j]=100;
                                        pruned = true;
                                        removed=1;
                                    }

                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (removed ==1)
                            {
                                tree[index].child_num-=1;
                                tree[sub_tree_nodes_index[child_index+i]].node = 100;
                                sub_tree_nodes[child_index+i]=100;
                            }
                            break;
                        case 5:
                            removed=0;
                            for (int j = i+1 ; j<child_num_temp ;j++)
                            {
                                if(sub_tree_nodes[child_index+j]== 4 || sub_tree_nodes[child_index+j]== 5
                                   || sub_tree_nodes[child_index+j]== 6 || sub_tree_nodes[child_index+j]== 7
                                   || sub_tree_nodes[child_index+j]== 2 || sub_tree_nodes[child_index+j]== 3
                                   || sub_tree_nodes[child_index+j]== 8 || sub_tree_nodes[child_index+j]== 18
                                   || sub_tree_nodes[child_index+j]== 19 || sub_tree_nodes[child_index+j]== 9)
                                {
                                    if ( sub_tree_nodes[child_index+j]==7)
                                    {
                                        tree[index].child_num-=1;
                                        tree[sub_tree_nodes_index[child_index+j]].node = 100;
                                        sub_tree_nodes[child_index+j]=100;
                                        pruned = true;
                                        removed=1;
                                    }

                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (removed ==1)
                            {
                                tree[index].child_num-=1;
                                tree[sub_tree_nodes_index[child_index+i]].node = 100;
                                sub_tree_nodes[child_index+i]=100;
                            }
                            break;
                        case 6:
                            removed=0;
                            for (int j = i+1 ; j<child_num_temp ;j++)
                            {
                                if(sub_tree_nodes[child_index+j]== 4 || sub_tree_nodes[child_index+j]== 5
                                   || sub_tree_nodes[child_index+j]== 6 || sub_tree_nodes[child_index+j]== 7
                                   || sub_tree_nodes[child_index+j]== 2 || sub_tree_nodes[child_index+j]== 3
                                   || sub_tree_nodes[child_index+j]== 8 || sub_tree_nodes[child_index+j]== 18
                                   || sub_tree_nodes[child_index+j]== 19 || sub_tree_nodes[child_index+j]== 9)
                                {
                                    if ( sub_tree_nodes[child_index+j]==4)
                                    {
                                        tree[index].child_num-=1;
                                        tree[sub_tree_nodes_index[child_index+j]].node = 100;
                                        sub_tree_nodes[child_index+j]=100;
                                        pruned = true;
                                        removed=1;
                                    }

                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (removed ==1)
                            {
                                tree[index].child_num-=1;
                                tree[sub_tree_nodes_index[child_index+i]].node = 100;
                                sub_tree_nodes[child_index+i]=100;
                            }
                            break;
                        case 7:
                            removed=0;
                            for (int j = i+1 ; j<child_num_temp ;j++)
                            {
                                if(sub_tree_nodes[child_index+j]== 4 || sub_tree_nodes[child_index+j]== 5
                                   || sub_tree_nodes[child_index+j]== 6 || sub_tree_nodes[child_index+j]== 7
                                   || sub_tree_nodes[child_index+j]== 2 || sub_tree_nodes[child_index+j]== 3
                                   || sub_tree_nodes[child_index+j]== 8 || sub_tree_nodes[child_index+j]== 18
                                   || sub_tree_nodes[child_index+j]== 19 || sub_tree_nodes[child_index+j]== 9)
                                {
                                    if ( sub_tree_nodes[child_index+j]==5)
                                    {
                                        tree[index].child_num-=1;
                                        tree[sub_tree_nodes_index[child_index+j]].node = 100;
                                        sub_tree_nodes[child_index+j]=100;
                                        pruned = true;
                                        removed=1;
                                    }

                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (removed ==1)
                            {
                                tree[index].child_num-=1;
                                tree[sub_tree_nodes_index[child_index+i]].node = 100;
                                sub_tree_nodes[child_index+i]=100;
                            }
                            break;

                    }
                }
                for (int i=0; i<child_num_temp  ;i++)
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
                    return decode_prune5(tree,index);
                }
            case 1:
                child_num_temp = tree[index].child_num;
                child_index = sub_tree_nodes.size()-child_num_temp;
                for (int i=0; i<child_num_temp  ;i++)
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
                    return decode_prune5(tree,index);
                }

        }
    }

    vector<blackboard::Node> decode_prune6 (vector<blackboard::Node> tree , int index)
    {
        int child_index;
        int condition_child=0;
        switch(tree[index].node) {
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
            case 18:
            case 19:
            case 20:
            case 21:
                sub_tree_nodes.push_back(tree[index].node);
                sub_tree_nodes_index.push_back(index);
                index = index - 1;
                return decode_prune6(tree, index);
            case 10:
            case 11:
            case 12:
            case 13:
            case 14:
            case 15:
            case 16:
            case 17:
            case 22:
            case 23:
            case 24:
                if (index ==1)
                {

                    tree[index].node = 100;
                    pruned = true;
                }
                sub_tree_nodes.push_back(tree[index].node);
                sub_tree_nodes_index.push_back(index);
                index = index - 1;
                return decode_prune6(tree, index);
            case 0: child_num_temp = tree[index].child_num;
                    child_index = sub_tree_nodes.size()-child_num_temp;
                    condition_child=0;
                for(int i =0; i<child_num_temp;i++)
                {
                    if(sub_tree_nodes[child_index+i]== 100)
                    {
                        tree[index].child_num-=1;
                    }
                }
                    if (index ==1)
                   {
                       for(int i =0; i<child_num_temp;i++)
                       {
                           if(sub_tree_nodes[child_index+i]== 10 || sub_tree_nodes[child_index+i]== 11 || sub_tree_nodes[child_index+i]== 12
                              || sub_tree_nodes[child_index+i]== 13 || sub_tree_nodes[child_index+i]== 14 || sub_tree_nodes[child_index+i]== 15
                              || sub_tree_nodes[child_index+i]== 16 || sub_tree_nodes[child_index+i]== 17 || sub_tree_nodes[child_index+i]== 22
                              || sub_tree_nodes[child_index+i]== 23 || sub_tree_nodes[child_index+i]== 24)
                           {
                               condition_child+=1;
                           }
                       }

                       if (condition_child==child_num_temp)
                       {
                           for(int i =0; i<child_num_temp;i++)
                           {
                               tree[sub_tree_nodes_index[child_index+i]].node = 100;
                           }
                           tree[index].node=100;
                           pruned = true;
                       }
                   }


                for (int i=0; i<child_num_temp  ;i++)
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
                    return decode_prune6(tree,index);
                }
            case 1: child_num_temp = tree[index].child_num;
                child_index = sub_tree_nodes.size()-child_num_temp;
                condition_child=0;
                for(int i =0; i<child_num_temp;i++)
                {
                    if(sub_tree_nodes[child_index+i]== 100)
                    {
                        tree[index].child_num-=1;
                    }
                }
                if (index ==1)
                {
                    for(int i =0; i<child_num_temp;i++)
                    {
                        if(sub_tree_nodes[child_index+i]== 10 || sub_tree_nodes[child_index+i]== 11 || sub_tree_nodes[child_index+i]== 12
                           || sub_tree_nodes[child_index+i]== 13 || sub_tree_nodes[child_index+i]== 14 || sub_tree_nodes[child_index+i]== 15
                           || sub_tree_nodes[child_index+i]== 16 || sub_tree_nodes[child_index+i]== 17 || sub_tree_nodes[child_index+i]== 22
                           || sub_tree_nodes[child_index+i]== 23 || sub_tree_nodes[child_index+i]== 24)
                        {
                            condition_child+=1;
                        }
                    }

                    if (condition_child==child_num_temp)
                    {
                        for(int i =0; i<child_num_temp;i++)
                        {
                            tree[sub_tree_nodes_index[child_index+i]].node = 100;
                        }
                        tree[index].node=100;
                        pruned = true;
                    }
                }


                for (int i=0; i<child_num_temp  ;i++)
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
                    return decode_prune6(tree,index);
                }

        }



    }

    vector<blackboard::Node> decode_prune7 (vector<blackboard::Node> tree , int index)
    {
        int child_index;
        switch(tree[index].node){
            case 2 ... 24:
                sub_tree_nodes.push_back(tree[index].node);
                sub_tree_nodes_index.push_back(index);
                index = index -1;
                return decode_prune7(tree,index);
            case 0:
                child_num_temp = tree[index].child_num;
                child_index = sub_tree_nodes.size()-child_num_temp;
                for(int i =0; i<child_num_temp-1;i++)
                {
                        if(sub_tree_nodes[child_index+i]== 4 || sub_tree_nodes[child_index+i]== 5
                           || sub_tree_nodes[child_index+i]== 6 || sub_tree_nodes[child_index+i]== 7
                           || sub_tree_nodes[child_index+i]== 2 || sub_tree_nodes[child_index+i]== 3
                           || sub_tree_nodes[child_index+i]== 8 || sub_tree_nodes[child_index+i]== 18
                           || sub_tree_nodes[child_index+i]== 19 || sub_tree_nodes[child_index+i]== 9)
                    {
                        for (int j = i+1 ; j<child_num_temp ;j++)
                        {
                            if(sub_tree_nodes[child_index+j]== 4 || sub_tree_nodes[child_index+j]== 5
                               || sub_tree_nodes[child_index+j]== 6 || sub_tree_nodes[child_index+j]== 7
                               || sub_tree_nodes[child_index+j]== 2 || sub_tree_nodes[child_index+j]== 3
                               || sub_tree_nodes[child_index+j]== 8 || sub_tree_nodes[child_index+j]== 18
                               || sub_tree_nodes[child_index+j]== 19 || sub_tree_nodes[child_index+j]== 9)
                            {
                                if (sub_tree_nodes[child_index+i]== sub_tree_nodes[child_index+j])
                                {
                                    tree[index].child_num-=1;
                                    tree[sub_tree_nodes_index[child_index+i]].node = 100;
                                    pruned = true;
                                    break;
                                }

                            }
                            else
                            {
                                break;
                            }
                        }


                    }
                }
                for (int i=0; i<child_num_temp  ;i++)
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
                    return decode_prune7(tree,index);
                }
            case 1:
                child_num_temp = tree[index].child_num;
                child_index = sub_tree_nodes.size()-child_num_temp;
                for (int i=0; i<child_num_temp  ;i++)
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
                    return decode_prune7(tree,index);
                }
        }
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
                            pruned = true;
                    }

                    if ((sub_tree_nodes[child_index+i]==0 || sub_tree_nodes[child_index+i]==1 ) && tree[sub_tree_nodes_index[child_index+i]].child_num <= 0) {
                        tree[index].child_num-=1;
                        tree[sub_tree_nodes_index[child_index+i]].node = 100;
                        pruned = true;
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
                        pruned = true;
                    }

                    if ((sub_tree_nodes[child_index+i]==0 || sub_tree_nodes[child_index+i]==1 ) && tree[sub_tree_nodes_index[child_index+i]].child_num <= 0) {

                        tree[index].child_num-=1;
                        tree[sub_tree_nodes_index[child_index+i]].node = 100;
                        pruned = true;
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



    vector<blackboard::Node> BT_reduction (vector<blackboard::Node> Btree)
    {


        vector<blackboard::Node> bt = Btree;
        for(int i=0; i<bt.size();i++)
        {
            Seq_type[i]=0;

        }



        bool pickObj, msgR, msgS=false;
        for (int t=0;t<bt.size();t++)
        {
            if(bt[t].node==18)
            {
                pickObj=true;
            }
            if(bt[t].node==21)
            {
                msgS=true;
            }
            if(bt[t].node==22)
            {
                msgR=true;
            }
        }
        //bt= decode_prune3(bt,bt.size()-1,pickObj,msgR,msgS); // remove dependent node if prior node not in the tree

        while(pruned && bt.size()>2)
        {
            pruned = false;
            if(bt.size()>2) {
                bt = decode_prune1(bt, bt.size() - 1); // replace single child with the child node
            }

            if(bt.size()>2) {
                bt = decode_prune0(bt, bt.size() - 1); // collapse multiple levels of the same non-leaf node
            }

            if(bt.size()>2)
            {
                bt= decode_prune2(bt,bt.size()-1); // remove nodes to the right of action node in the sel tree
            }
            if( bt.size()>2) {
                bt= decode_prune4(bt,bt.size()-1); // remove duplicate condition nodes

            }

            if( bt.size()>2) {
                bt= decode_prune7(bt,bt.size()-1); // remove duplicate action nodes

            }
            if( bt.size()>2) {
                bt= decode_prune5(bt,bt.size()-1);// remove action nodes with cancel effect

            }

        }



        if (bt[0].child_num==1 && (bt[1].node ==1 || bt[1].node ==0 ))
        {
            bt.erase(bt.begin());
        }// replace sel with seq if seq in the only child

        pruned =true;
        while(pruned && bt.size()>2) {
            pruned = false;
            bt= decode_prune6(bt,bt.size()-1); // remove conditions on the right side of the tree

        }


         if (bt[0].child_num==1 && (bt[1].node ==1 || bt[1].node ==0 ))
         {
             bt.erase(bt.begin());
         }// replace sel with seq if seq in the only child



        pruned =true;

        return  bt;
    }



    void reset_BT ()
    {
        BT.clear();
        sub_root=0;
        subtree_size=0;
        read_msg_node=0;
        send_msg_node=0;
        final_child=10 ;
        node_index = 0;
    }

    void set_nodes_list (vector<blackboard::Node>  BTree)
    {
        for (int i=0; i<BTree.size(); i++)
        {
            if(BTree[i].node == 16 || BTree[i].node==17)
            {
                nodes_list.push_back(16);
                nodes_list.push_back(17);
            }

            if(BTree[i].node == 18 || BTree[i].node==19)
            {
                nodes_list.push_back(18);
                //nodes_list.push_back(19);
            }

            if(BTree[i].node == 20)
            {
                nodes_list.push_back(20);
            }

            if(BTree[i].node == 21 || BTree[i].node==22)
            {
                nodes_list.push_back(21);
                nodes_list.push_back(22);

            }
        }

    }



    void Original_bts_reduction ()
    {
        for(int i=0;i<original_Bts.size();i++)
        {
            original_Bts_r.push_back(BT_reduction(original_Bts[i]));
        }
    }

    void Extracted_bts_reduction ()
    {
        for(int i=0;i<extracted_Bts.size();i++)
        {
            extracted_Bts_r.push_back(BT_reduction(extracted_Bts[i]));
        }
    }


    bool check_BT_motion (vector<blackboard::Node>  BTree)
    {
        sm.reset(new Swarm);
        sm->reset();
        sm->set_BT(BTree);
        sm->generate_swarm();
        sm->simulate_swarm();

        float average_Velx = 0.0;
        float average_Vely = 0.0;

        for (int i=1;i<blackboard::simulation_timesteps;i++)
        {
            float average_x = 0.0;
            float average_y = 0.0;
            for (int j=0; j< blackboard::Swarm_size;j++)
            {
                average_x = average_x + (sm->SwarmX[i*blackboard::Swarm_size+j]-sm->SwarmX[(i-1)*blackboard::Swarm_size+j]);
                average_y = average_y + (sm->SwarmY[i*blackboard::Swarm_size+j]-sm->SwarmY[(i-1)*blackboard::Swarm_size+j]);
            }
            average_Velx = average_Velx + (average_x/blackboard::Swarm_size);
            average_Vely = average_Vely + (average_y/blackboard::Swarm_size);
        }
        average_Velx = average_Velx / (blackboard::simulation_timesteps-1);
        average_Vely = average_Vely / (blackboard::simulation_timesteps-1);

        sm.reset();

        //BT_Nodes_tick_times= sm.controller.blackboard.Nodes_tick_times;



        if(average_Velx == 0 && average_Vely == 0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }




    void Initialize_originals ()
    {

//        for (int i =0; i<test_size ; i++)
//        {
//            RandomBT_generator(2);
//            original_Bts.push_back(BT);
//            reset_BT();
//        }
        vector<blackboard::Node>  extracted_BT;
        int i =0;
        while (i<test_size)
        {
            cout<<"BT "<<i<<endl;
            RandomBT_generator(2);
            if(check_BT_motion(BT))
            {

                //original_Bts.push_back(BT);
                extracted_BT = BT_reduction(BT);
                if(check_BT_motion(extracted_BT))
                {


                    original_Bts_r.push_back(extracted_BT);
                    i=i+1;
                }

            }
            reset_BT();
        }

        cout<<"Original BTs Created";
        cout<<endl;
    }

    void Controller_extraction (bool sim_data)
    {
        vector<blackboard::Node>  extracted_BT;
        unique_ptr<BTevolution> EA;
        unique_ptr<BTevolution_IslandModel> EA_IM;
        vector<int> full_Bts;
        nodes_list = {2,3,4,5,6,7,8,9,13,16,17,18,19,23};
        //nodes_list = {2,3,4,5,6,7,8,9,13};

        for (int i =36; i<37 ; i++)
        {
            //unsigned seed = std::time(0) + std::rand();
            //std::srand(seed);

            cout<<"**********************";
            cout<<"BT no : ";
            cout<<i+1;
            cout<<endl;


            //EA.reset (new BTevolution  (original_Bts[i],85,50,nodes_list));
            EA.reset (new BTevolution  (original_Bts[i],85,50,nodes_list));
            EA->object_in_env=true;
            EA->areas_in_env= true;
            EA->fixed_env=true;
            EA->Evolution();
            //EA->compare_agents_metrics(i,original_Bts[i]);
            if (sim_data)
            {
                    extracted_BT=BT_reduction(EA->BestBT);
                    for (int g=0;g<EA->generations_number;g++)
                    {
                        EA->all_BestBT[g]=BT_reduction(EA->all_BestBT[g]);
                        EA->all_worstBT[g]=BT_reduction(EA->all_worstBT[g]);
                    }
                    cout<<"Reduction Done"<<endl;
                    EA->save_final_sim_data(i, extracted_BT);
                    //EA->save_final_sim_data_ghost_swarm(i, extracted_BT);
                    cout<<"file saved"<<endl;
            }



            EA.reset();
        }
        cout<<"**********************";
        cout<<endl;
        cout<<"Imitation Done";
        cout<<endl;
        cout<<"**********************";
        cout<<endl;



    }



    void reprune_BT (int i)
    {
        vector<blackboard::Node>  BT;
        vector<blackboard::Node>  extracted_BT;
        rapidcsv::Document doc(sub_folder+imitated_BT_Best10_"+to_string(i)+".csv", rapidcsv::LabelParams(-1, -1));

        vector<int> nodes    = doc.GetRow<int>(0);
        vector<int> childNum = doc.GetRow<int>((1));

        for(int j=0; j<nodes.size(); j++)
        {
            BT.push_back({nodes[j],childNum[j]});
        }

        extracted_BT=BT_reduction(BT);

        std::ofstream myfile;
        myfile.open (sub_folder+imitated_BT"+to_string((i))+".csv");
        for ( int j=0; j<extracted_BT.size(); j++) {
            myfile << extracted_BT[j].node << ",";

        }
        myfile << "\n";
        for ( int j=0; j<extracted_BT.size(); j++) {
            myfile << extracted_BT[j].child_num << ",";
        }
        myfile.close();


    }

    void read_originalBTs ()
    {
        rapidcsv::Document doc(sub_folder+originals/OriginalBTsEvn3.csv", rapidcsv::LabelParams(-1, -1));

        original_Bts.clear();
        vector<blackboard::Node> BT;

        for ( int i=0; i<100; i+=2)
        {
            vector<int> nodes    = doc.GetRow<int>(i);
            vector<int> childNum = doc.GetRow<int>((i+1));


            for(int j=0; j<nodes.size(); j++)
            {
                BT.push_back({nodes[j],childNum[j]});
            }
            original_Bts.push_back(BT);
            BT.clear();
        }
    }


    void save_originalBTs ()
    {
        std::ofstream myfile;
        myfile.open (sub_folder+"/BTsC1.csv");
        for ( int i=0; i<test_size; i++)
        {
            for(int j=0; j<original_Bts_r[i].size(); j++)
            {    myfile << original_Bts_r[i][j].node;
                if (j!=original_Bts_r[i].size()-1)
                {
                    myfile<< ",";
                }
            }
            myfile <<"\n";
            for(int j=0; j<original_Bts_r[i].size(); j++)
            {            myfile << original_Bts_r[i][j].child_num;
                if (j!=original_Bts_r[i].size()-1)
                {
                    myfile<< ",";
                }
            }
            myfile <<"\n";

        }
        myfile.close();
    }


};

#endif //MAIN_CPP_MODELTEST_H
