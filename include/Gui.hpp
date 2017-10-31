#ifndef GUI_HPP
#define GUI_HPP

#include "State.hpp"
#include "Map.hpp"
#include "Utils.hpp"

class GUI{
public:
    Size display_size;
    Mat display;

    GUI(int rows, int cols);
    void drawCar(State src);
    void drawObs(Map map);
    void markPoint(int i, int j);
    void show(int t);
};

#endif