#include "AStar.hpp"
#include <algorithm>
#include <iostream>
#include <math.h>
#include <chrono>
#include <vector>
#include <unordered_map>
#include <random>
#include "/usr/local/opt/libomp/include/omp.h"
#include <cstdint>
using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

//void AStar::Generator::CalcHash(uint stateSpaceSize)
//{
  //  uint shift = 10 % (8 * .bit.length()
//}

void AStar::Generator::ZobristTable(uint x1, uint x2, uint y1, uint y2,uint stateSpaceSize)
{
   std::unordered_map<int, uint64_t> zobristTable;
   std::default_random_engine generator(42);
   std::uniform_int_distribution<uint64_t> distribution(0, std::numeric_limits<uint64_t>::max());
   for (int state = 0; state < stateSpaceSize; state++) {
    // Considering only the x cordinate as the abstract feature
         if (x1 == 1) 
         {
            zobristTable[state] = distribution(generator);
         }
    }
    for (int state = 0; state < stateSpaceSize; state++) {
        std::cout << "State " << state << ": " << zobristTable[state] << std::endl;
    }
}
void AStar::Generator::clearCollisions()
{
    walls.clear();
}
int hashfunc (int value, int NumThreads)
{
    return value%NumThreads;
}
AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
{
   auto start = std::chrono::high_resolution_clock::now();
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    openSet.reserve(100);
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));

    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        current = *current_it;
        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        }
        if (current->coordinates == target_) {
            break;
        }
        closedSet.push_back(current);
        openSet.erase(current_it);
  
 const int numThreads = 8;
 #pragma omp parallel num_threads(numThreads)
    {
        int threadID = omp_get_thread_num();
        #pragma omp for
        for (uint i = 0; i < directions; ++i) {
            if (hashfunc(i, numThreads) == threadID) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }
            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
            }
    }
        }
    }
    auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{

    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
///usr/local/opt/llvm/bin/clang++ -fopenmp main.cpp   
