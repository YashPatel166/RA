#include <iostream>
#include "source/AStar.hpp"
#include "source/AStar.cpp"
#include <iomanip>
#include <chrono>

#include <stack>
#include <set> 
#include <thread>
#include <ctime>
#include <vector>
#include <cstdlib>
#include <sys/time.h>
#include "/usr/local/opt/libomp/include/omp.h"

int main()
{
 
    AStar::Generator generator;
    auto start = std::chrono::high_resolution_clock::now();
    generator.setWorldSize({10000, 10000});
    generator.initializeConstants();
    generator.setDiagonalMovement(false);
     auto stop = std::chrono::high_resolution_clock::now();
	double duration1 = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count();

    std::cout<<"Time taken for initialization:"<<duration1<<"\n";
    generator.setHeuristic(AStar::Heuristic::euclidean);
   
    std::cout << "Generate path ... \n";
    int x1, y1, x2, y2;
    std::cout<<"Enter X1 and Y1:\n";
    std::cin>>x1>>y1;
    std::cout<<"Enter X2 and Y2:\n";
    std::cin>>x2>>y2;
    
    auto path = generator.findPath({x1,y1}, {x2,y2});

    //for(auto& coordinate : path) {
     //  std::cout << "("<< coordinate.x << "," << coordinate.y << ") -> ";
    //}




   
}
