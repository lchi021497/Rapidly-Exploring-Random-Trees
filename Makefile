app: rrt.o kdtree.o
	g++ rrt.o kdtree.o -o sfml-app -lsfml-graphics -lsfml-window -lsfml-system -fopenmp

kd-tree-test: kdtree.o kdtree-test.cpp
	g++ -std=c++11 -g kdtree.o -o kd-tree-test kdtree-test.cpp

kdtree.o: kdtree.cpp kdtree.hpp
	g++ -std=c++11 -g -c kdtree.cpp

rrt.o:
	g++ -g -std=c++11 geometry.h kdtree.hpp kdtree.cpp -c rrt.cpp -fopenmp

.PHONY: clean
clean:
	rm *.o sfml-app
clean-test:
	rm kdtree.o rm kd-tree-test