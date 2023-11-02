#include <iostream>
#include "source/AStar.hpp"
#include "source/AStar.cpp"
#include <iomanip>
#include <chrono>

#include <stack>
#include <set> 
#include <thread>
#include <chrono>
#include <vector>
#include "/usr/local/opt/libomp/include/omp.h"

int main()
{
    //auto start = std::chrono::high_resolution_clock::now();
    AStar::Generator generator;
    generator.setWorldSize({15, 15});
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    std::cout << "Generate path ... \n";
    auto path = generator.findPath({3,6}, {0, 3});

    for(auto& coordinate : path) {
        std::cout << "("<< coordinate.x << " " << coordinate.y << ") -> ";
    }
     std::cout<<"\n";
 //  auto stop = std::chrono::high_resolution_clock::now();
	//auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
	
    //std::cout <<"\n"<<"Time taken by function: "<< duration.count() << " microseconds \n";
}