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
#include <chrono>
#include <cstdlib>
#include <sys/time.h>
#include "/usr/local/opt/libomp/include/omp.h"

int main()
{

 struct timeval startTime, stopTime;
 gettimeofday(&startTime, nullptr);

   // clock_t start = clock();


 
    AStar::Generator generator;
    generator.setWorldSize({1000, 1000});
    generator.setHeuristic(AStar::Heuristic::euclidean);
   
    std::cout << "Generate path ... \n";
    int x1, y1, x2, y2;
    std::cout<<"Enter X1 and Y1:\n";
    std::cin>>x1>>y1;
    std::cout<<"Enter X2 and Y2:\n";
    std::cin>>x2>>y2;
    
    auto path = generator.findPath({x1,y1}, {x2,y2});

    for(auto& coordinate : path) {
        std::cout << "("<< coordinate.x << "," << coordinate.y << ") -> ";
    }


gettimeofday(&stopTime, nullptr);
    double duration = (stopTime.tv_sec - startTime.tv_sec) + (stopTime.tv_usec - startTime.tv_usec) / 1e6;

    std::cout << "Time taken by function: " << duration << " seconds\n";

   /*clock_t stop = clock();
  
    double duration = (double)(stop - start) / CLOCKS_PER_SEC;

    std::cout << "Time taken by function: " << duration << " seconds\n";
     std::cout<<"\n";*/
}
