//
//  AStar.hpp
//  AStar
//
//  Created by Edvinas on 03/11/2017.
//  Copyright © 2017 Edvinas. All rights reserved.
//

#ifndef AStar_hpp
#define AStar_hpp

#include <stdio.h>
#include <set>
#include <vector>

namespace AStar {
    
    struct PositionMap {
        double x, y;
        PositionMap(double x, double y);
    };

    struct Position {
        int x, y;
        bool operator == (const Position& other);
        Position(int x, int y);
    };
    
    struct Node {
        Node *parent;
        Position position;
        
        int costToReach; // AKA G
        int costToGoal; // AKA H
        Node(Position position, Node *parent = nullptr);
        
        const int totalCost() const;
        const int estimatedDistance(const Position &destination);//{
    };
    
    struct Map {
        Map(int height, int width, std::vector<int> data, int emptyCell, PositionMap origin, double cellResolution);
        // Inflate all obsticles by given radius
        void inflateObsticles(double robotWidth, double robotHeight);
        void debugMap();
        void expandAroundException(int x, int y, int radius, bool exceptions[]);
        bool obstacleExists(const Position &position);
        Position transformMapPositionToGridPosition(const PositionMap &positionInMap);
        
        std::vector<signed char> getFlattenedMap();

        int height, width;
        PositionMap origin;
        double cellResolution;

        private:
            int emptyCell = 100;
            std::vector<std::vector<int>> mapVector;
    };
    
    class AStarAlgorithm {
    public:
        AStarAlgorithm(Map map);
        std::vector<Position> findPath(const Position &startPosition, const Position &goalPosition);
    private:
        Map map;
        /* In how many directions try to expand path
         8: Goes in all directions (diagonal included)
         4: Goes only to sides
         */
        int directions = 8;
        
        /* Convienience array to calculate neighbours index,
         last 4 positions are diagonal
         */
        int direction[8][2] = {
            { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
            { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
        };
        
        Node* findNeighbour(std::vector<Node*> &nodeHeap, Position &position);
        void freeMemory(std::vector<std::vector<Node*>> nodesCache);
    };
}

#endif /* AStar_hpp */

