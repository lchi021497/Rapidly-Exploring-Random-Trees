app: rrt.o
	g++ rrt.o -o sfml-app -lsfml-graphics -lsfml-window -lsfml-system

kd-tree-test: kdtree.o kdtree-test.cpp
	g++ -std=c++11 -g kdtree.o -o kd-tree-test kdtree-test.cpp

kdtree.o: kdtree.cpp kdtree.hpp
	g++ -std=c++11 -g -c kdtree.cpp

rrt.o:
	g++ -std=c++11 -I/usr/local/include geometry.h -c rrt.cpp

.PHONY: clean
clean:
	rm *.o sfml-app
clean-test:
	rm kdtree.o rm kd-tree-test