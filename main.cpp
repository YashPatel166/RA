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
    AStar::Generator generator;
    generator.setWorldSize({15, 15});
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);
    std::cout << "Generate path ... \n";
    int x1, y1, x2, y2;
    std::cout<<"Enter X1 and Y1:\n";
    std::cin>>x1>>y1;
    std::cout<<"Enter X2 and Y2:\n";
    std::cin>>x2>>y2;
    generator.ZobristTable(x1, y1, x2, y2, 100);
    auto path = generator.findPath({x1,y1}, {x2,y2});

    for(auto& coordinate : path) {
        std::cout << "("<< coordinate.x << "," << coordinate.y << ") -> ";
    }
     std::cout<<"\n";
}
