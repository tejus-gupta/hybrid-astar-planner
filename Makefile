Sources=src/State.cpp src/Map.cpp src/Gui.cpp src/Compare.cpp src/Planner.cpp test.cpp
Target = astar

# general compiler settings
CXXFLAGS = -O3 -ffast-math -w
LIBS = `pkg-config --libs opencv`

all:
	$(CXX) $(CXXFLAGS) $(Sources) $(LIBS) -o $(Target) $(LDFLAGS)

clean:
	@$(RM) $(Target)

.PHONY: clean