#include <iostream>
using namespace std;
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include<math.h>
#include <stack>
#include <fstream>
#include <random>
#include "Swarm.h"
#include "AntsSwarm.h"
#include "SimMetrics.h"
#include "SwarmMetrics.h"
#include "BTevolution.h"
#include "AntBTevolution.h"
#include "AntBTevolutionPP.h"
#include "ModelTest.h"
#include "Env_Evolution.h"
#include <stdio.h>
#include <stdint.h>
#include <unordered_map>
#include <set>
#include "BehaviorTree.h"
#include <random>
#include "VideoTracker.h"
#include "AntModelTest.h"

int main(int argc, char *args[]) {

    srand(time(0));

    ModelTest m1(1);
    //m1.Initialize_originals();
    //m1.save_originalBTs();
    m1.read_originalBTs();
    m1.Controller_extraction(true);


    return 0;
   }
