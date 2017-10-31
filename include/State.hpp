#ifndef STATE_HPP
#define STATE_HPP

#include "Utils.hpp"

class State{
public:
    float x;
    float y;
    float theta;

    //gx, gy and gtheta are co-ordinates in the 80X80 grid
    int gx;
    int gy;
    int gtheta;

    //for running dijkstra
    int dx;
    int dy;

    float cost2d;
    float cost3d;

    float change;
    float velocity;

    State* next;
    State* previous;

    State();
    State(float x, float y, float theta);
    vector<State> getNextStates();
};

#endif
