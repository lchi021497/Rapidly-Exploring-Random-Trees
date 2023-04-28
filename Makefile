app: rrt.o
	g++ -std=c++11 -g rrt.o kdtree.o -o sfml-app -lsfml-graphics -lsfml-window -lsfml-system 

kd-tree-test: kdtree.o kdtree-test.cpp
	g++ -std=c++11 -g kdtree.o -o kd-tree-test kdtree-test.cpp

demo: kdtree.o demo_kdtree.cpp
	g++ -std=c++11 -g kdtree.o -o demo_kdtree demo_kdtree.cpp

kdtree.o: kdtree.cpp kdtree.hpp
	g++ -std=c++11 -g -c kdtree.cpp

rrt.o: kdtree.hpp kdtree.cpp 
	g++ -g -std=c++11 geometry.h -c rrt.cpp kdtree.hpp kdtree.cpp 

.PHONY: clean
clean:
	rm *.o sfml-app
clean-test:
	rm kdtree.o rm kd-tree-test
